#! /usr/bin/env python3

'''
This code reads positions of all robots, write positions and goals (random or custom) to cbs_input.yaml,
calls cbs.py planner. This code also executes the plan that was written in cbs_output.yaml by cbs.py
'''

import os
import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
import re
import math
from tf_transformations import euler_from_quaternion
import random
import time
import yaml
import ament_index_python.packages as packages
import cbs

CUSTOM_GOALS = True # True: read positions from /params/custom_goals.yaml False: Random targets
REPLAN = True # True: Plan and execute continuosly False: Plan and execute once
THRE_ROBOT_ON_TARGET = 0.1 # Threshold to consider that robot has reached a target
TARGETS_RANDOM_POOL_SIZE = 3 # Size of target area in meters (Gazebo squares)
KP_linear = 0.25 # Controller "proportional" gain for linear velocity
KP_angular = 0.5 # Controller "proportional" gain for angular velocity
KI_linear = 0.05 # Controller "integral" gain for angular velocity
KI_angular = 0.01 # Controller "integral" gain for angular velocity
MAX_LINEAR_VELOCITY = 1
MAX_ANGULAR_VELOCITY = 0.2
DISCRETIZATION = 2 # Discretization of map (1: one point per gazebo square, 2: 4 points per gazebo square)
SHIFT_MAP = 10*DISCRETIZATION # CBS only accepts positive integer values, all the values used by the CBS are shifted beforehand. 10 guarantees that if robots are inside gazebo dafault plane, all points sent to CBS are positive
CBS_MAP_DIMENSION = 20*DISCRETIZATION # This calculate the whole gazebo default area

class Planner(Node):
    def __init__(self):
        super().__init__('robot_controller')      
        self.number_of_robots = int(self.count_robot_topics())        
        self.robots = ['robot{}'.format(i) for i in range(self.number_of_robots)]
        self.subscribers = []
        self.robot_publishers = []
        self.alarm_publisher = ''
        self.positions = [Point() for _ in range(len(self.robots))]
        self.orientations = [float for _ in range(len(self.robots))]
        self.distance_to_target = [float for _ in range(len(self.robots))]
        self.final_goal = [Point() for _ in range(len(self.robots))]
        self.position_received = 0
        self.first_time_planning = True
        self.cbs_time_schedule = 1
        self.number_of_succesfully_executed_plans = 0
        # Initialize the integral and previous error for PID controller
        self.angular_integral = [0.0] * self.number_of_robots
        self.distance_integral = [0.0] * self.number_of_robots
        
        self.obstacles=(500, 500) # Dummy obstacle (CBS code needs at least one obstacle, if no obstacle is defined, it will consider this obstacle far away that do not interfere)
        self.read_obstacle_param()
        self.create_all_subscribers()
        self.create_all_publishers()
        self.generate_new_targets()
        
    def count_robot_topics(self):
        '''
        Count number of robot topics to know the number of robots avoiding hardcoding it here. Return an integer
        '''
        topic_list = Node.get_topic_names_and_types(self)
        robot_count = 0
        for item in topic_list:
            if item[0].startswith('/robot') and item[0].endswith('/cmd_vel'):
                robot_count += 1        
        return robot_count
        
    def create_all_subscribers(self):
        '''
        Create one subscriber per robot
        '''
        for robot_name in self.robots:
            topic = f'/{robot_name}/odom'
            subscriber = self.create_subscription(Odometry, topic, lambda msg, topic=topic: self.position_callback(msg, topic), 10)
            subscriber.robot_name = robot_name
            self.subscribers.append(subscriber)
    
    def create_all_publishers(self):
        '''
        Create one publishe per robot and one alarm publisher
        '''
        for robot_name in self.robots:
            topic = f'/{robot_name}/cmd_vel'
            publisher = self.create_publisher(Twist, topic, 10)
            self.robot_publishers.append(publisher)
        alarm_topic = '/planning_alarm'
        self.alarm_publisher = self.create_publisher(String, alarm_topic, 10)

    def position_callback(self, msg, topic):
        '''
        Receives the position of each robot and call the controller
        '''
        robot_index = re.search(r'/robot(\d+)/odom', topic).group(1)
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        (_, _, theta)  = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w]) # get orientation of robots from quaternion
        self.positions[int(robot_index)] = Point(x=position.x, y=position.y, z=position.z)
        self.orientations[int(robot_index)] = theta
        
        # Compute a new control input for every update in position
        self.drive_robots_to_cbs_waypoints()
        
    def all_robots_arrived_cbs_final_waypoint(self):
        '''
        Check if all robots arrived in cbs last waypoint, i.e, the goal. Return True only if all robots have arrived
        '''
        self.angular_integral = [0.0] * self.number_of_robots # Zero controller errors (Anti Windup)
        self.distance_integral = [0.0] * self.number_of_robots # Zero controller errors (Anti Windup)
        number_of_robots_in_target = 0
        cbs_last_waypoints = []
        for robot_index, _ in enumerate(self.target_waypoints):
            cbs_last_waypoints.append(self.target_waypoints[robot_index][-1])
        for robot_index, _ in enumerate(self.robots):
            current_position = self.positions[robot_index]
            # Points are shifted (SHIFT_MAP) by a fixed amount so that they are positive and are shrinked/inflated (DISCRETIZATION) to allow the use of CBS that only accepts positive integers
            self.distance_to_target[int(robot_index)] = self.get_distance((cbs_last_waypoints[int(robot_index)].x - SHIFT_MAP)/DISCRETIZATION - current_position.x, (cbs_last_waypoints[int(robot_index)].y - SHIFT_MAP)/DISCRETIZATION - current_position.y)
            if self.distance_to_target[int(robot_index)] < THRE_ROBOT_ON_TARGET:
                number_of_robots_in_target += 1 
        if int(number_of_robots_in_target) == int(len(self.robots)): # All robots on last waypoint
            self.halt_robots()
            self.number_of_succesfully_executed_plans += 1
            self.get_logger().warning(f'Number of succesfully executed plans: {self.number_of_succesfully_executed_plans}')
            self.cbs_time_schedule = 1 # Plan again
            return True
        else:
            return False        
    
    def all_robots_arrived_in_waypoints(self):
        '''
        Check if all robots arrived in a (mid) target. Return True only if all robots have arrived
        '''
        number_of_robots_in_waypoints = 0
        for robot_index, _ in enumerate(self.robots):
            current_position = self.positions[robot_index]
            target_waypoint  = self.get_next_target_waypoint(robot_index)
            # Points are shifted (SHIFT_MAP) by a fixed amount so that they are positive and are shrinked/inflated (DISCRETIZATION) to allow the use of CBS that only accepts positive integers
            self.distance_to_target[int(robot_index)] = self.get_distance((target_waypoint.x -SHIFT_MAP)/DISCRETIZATION - current_position.x, (target_waypoint.y-SHIFT_MAP)/DISCRETIZATION - current_position.y)
            if self.distance_to_target[int(robot_index)] < THRE_ROBOT_ON_TARGET:
                number_of_robots_in_waypoints += 1 
        if int(number_of_robots_in_waypoints) == int(len(self.robots)):
            self.get_logger().info(f'Robots Arrived in waypoint: {self.cbs_time_schedule}')
            return True
        else:
            return False
        
    def drive_robots_to_cbs_waypoints(self):
        '''
        Drive robots to the waypoints that are solution of the CBS planner. 
        When the target is reached, another random target is acquired and a new plan is generated
        Do this continuously
        '''
        if self.all_positions_received():
            self.call_cbs_planner()
                           
        if self.first_time_planning == False: # Planner was already called at least once
            if self.all_robots_arrived_cbs_final_waypoint(): # all robots arrived -> new targets -> call planner
                if REPLAN == True:
                    self.get_logger().info('All robots arrived on targets')
                    self.get_logger().info('Acquiring new targets')
                    self.generate_new_targets()
                    self.call_cbs_planner()
                else:
                    self.get_logger().warning(f'Plan Executed sucessfully!')
                    sys.exit()
            else: #Drive robots to next waypoint
                if self.all_robots_arrived_in_waypoints():
                    if self.cbs_time_schedule < max(self.max_cbs_times):
                        self.cbs_time_schedule += 1 # Next time in CBS schedule
                        self.get_logger().info(f'Going to waypoint: {self.cbs_time_schedule} | of total: {max(self.max_cbs_times)}')
                for robot_index, robot_name in enumerate(self.robots):  
                    target_waypoint = self.get_next_target_waypoint(robot_index)
                    current_orientation = self.orientations[robot_index]             
                    # Validate current_orientation
                    if isinstance(current_orientation, (int, float)):
                        current_orientation = float(current_orientation)
                    else:
                        error_msg = "Invalid current_orientation: " + str(current_orientation)
                        continue
                    self.command_robot(robot_index, target_waypoint, current_orientation)
                            

    def command_robot(self, robot_index, target_waypoint, current_orientation):
        '''
        Control the robot to the desired target_waypoint
        '''
        current_position = self.positions[robot_index]
        cmd_vel = Twist()
        try:
            # Points are shifted (SHIFT_MAP) by a fixed amount so that they are positive and are shrinked/inflated (DISCRETIZATION) to allow the use of CBS that only accepts positive integers
            target_x_scaled = (target_waypoint.x - SHIFT_MAP)/DISCRETIZATION
            target_y_scaled = (target_waypoint.y - SHIFT_MAP)/DISCRETIZATION
            error_x = target_x_scaled - current_position.x
            error_y = target_y_scaled - current_position.y

            target_angle = math.atan2(error_y, error_x) # Use atan2 to get the correct angle
                    
            # Calculate the closer angle taking into account the circular nature of angles
            angle_diff = target_angle - current_orientation
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi
                
            # PI control for rotation
            angular_error = angle_diff
            self.angular_integral[robot_index] += angular_error
            angular_output = KP_angular * angular_error + KI_angular * self.angular_integral[robot_index]
                
            # Set control mode based on the magnitude of the angle difference
            ANGLE_THRE = 0.4 # Too small -> Jittering, too big -> Robots move in a straight line with wrong direction
            if abs(angle_diff) > ANGLE_THRE:
                # Rotate in place towards the target
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = angular_output
            else:
                # PI control for translation
                distance_error = math.sqrt(error_x**2 + error_y**2)
                self.distance_integral[robot_index] += distance_error
                distance_output = KP_linear * distance_error + KI_linear * self.distance_integral[robot_index]

                # Move straight towards the target
                cmd_vel.linear.x = distance_output
                cmd_vel.angular.z = 0.0

            self.robot_publishers[int(robot_index)].publish(cmd_vel)
        except Exception as e:
            error_msg = "Error occurred during computation: " + str(e)
            self.get_logger().error(error_msg)   
    
    def halt_robots(self):
        '''
        Send zero to the robots if they are not supposed to move, avoid robots wander around
        '''
        cmd_vel = Twist()
        for robot_index, _ in enumerate(self.robots):
            try:
                cmd_vel.linear.x = float(0)
                cmd_vel.linear.y = float(0)
                cmd_vel.linear.z = float(0)
                cmd_vel.angular.x = float(0)
                cmd_vel.angular.y = float(0)
                cmd_vel.angular.z = float(0)
                self.robot_publishers[int(robot_index)].publish(cmd_vel)
            except Exception as e:
                error_msg = "Error halting robots: " + str(e)
                self.get_logger().error(error_msg)     

    def get_next_target_waypoint(self, robot_index):
        '''
        Get the next target waypoint in the CBS schedule, if it exists
        '''
        if len(self.target_waypoints[robot_index]) > self.cbs_time_schedule: # While new CBS waypoints exists -> Go to them
            target_waypoint = self.target_waypoints[robot_index][self.cbs_time_schedule]
        else:
            target_waypoint = self.target_waypoints[robot_index][self.max_cbs_times[robot_index]] # Reached its final waypoint -> Stay put
        return target_waypoint

    def generate_new_targets(self):
        '''
        Generate new targets (random or custom)
        '''
        if CUSTOM_GOALS == False:
            self.generate_new_random_targets()
        else:
            self.get_logger().info(f'Custom goals')
            self.read_and_set_custom_goals()

    def generate_new_random_targets(self):
        '''
        Generate one new random target for each robot inside a square centered at (0,0) with size TARGETS_RANDOM_POOL_SIZE
        '''
        for robot_index, _ in enumerate(self.robots):
            self.final_goal[int(robot_index)] = Point(x=float(random.randint(-TARGETS_RANDOM_POOL_SIZE*DISCRETIZATION, TARGETS_RANDOM_POOL_SIZE*DISCRETIZATION)), y=float(random.randint(-TARGETS_RANDOM_POOL_SIZE*DISCRETIZATION, TARGETS_RANDOM_POOL_SIZE*DISCRETIZATION)), z=0.01) # Random target positions
        while self.check_equal_points(self.final_goal) == True: # Unique targets
            self.generate_new_random_targets()     
        self.resolve_obstacle_conflicts()

    def check_equal_points(self, points):
        '''
        Check if target are the same
        '''
        for i in range(len(points)):
            for j in range(i + 1, len(points)):
                if points[i] == points[j]:
                    return True  # Found equal points
        return False  # No equal points found

    def write_data_to_yaml(self):       
        '''
        Write robots positions, obstacles and goals to the file that will be used as an input to the planner
        '''
        input_filename = 'scripts/params/cbs_input.yaml'
        filename = self.get_full_filename(input_filename)
        
        data = {'robots': [],
                "map": {
                "dimensions": [CBS_MAP_DIMENSION, CBS_MAP_DIMENSION],
                "obstacles": self.obstacles}}
        for robot_index, robot_name in enumerate(self.robots):
            start_x = int(round(self.positions[int(robot_index)].x*DISCRETIZATION))
            start_y = int(round(self.positions[int(robot_index)].y*DISCRETIZATION))
            goal_x = int(round(self.final_goal[int(robot_index)].x))
            goal_y = int(round(self.final_goal[int(robot_index)].y))
            agent_data = {'start': [(start_x + SHIFT_MAP), (start_y + SHIFT_MAP)], 'goal': [goal_x + SHIFT_MAP, goal_y + SHIFT_MAP], 'name': robot_name}
            data['robots'].append(agent_data)

        with open(filename, 'w') as file:
            yaml.dump(data, file, default_flow_style=None, indent=4)
            
    def read_obstacle_param(self):
        '''
        Read obstacles from custom_obstacles.yaml'
        '''
        input_filename = 'scripts/params/custom_obstacles.yaml'
        filename = self.get_full_filename(input_filename)
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        self.obstacles=[]
        for data in data['obstacles']:
            obst = data['obstacle']
            self.obstacles.append((int(obst[0]*DISCRETIZATION + SHIFT_MAP), int(obst[1]*DISCRETIZATION + SHIFT_MAP)))
    
    def read_and_set_custom_goals(self):
        '''
        Set goals read from file and check their validity
        '''
        input_filename = 'scripts/params/custom_goals.yaml'
        filename = self.get_full_filename(input_filename)
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        data_robots = data['robots']
        for robot_index, robot_data in enumerate(data_robots):
            if robot_index < self.number_of_robots:
                goal = robot_data['goal']
                self.final_goal[int(robot_index)].x = float(round(goal[0]))
                self.final_goal[int(robot_index)].y = float(round(goal[1]))
        # self.get_logger().info(f"Goals: {self.final_goal}")
        self.resolve_goal_conflicts()
        self.resolve_obstacle_conflicts()
                   
    def update_goal(self, robot_id, new_goal):
        '''
        Set goals read from file and check their validity
        '''
        self.final_goal[robot_id] = new_goal
        self.get_logger().info(f"Updated Goal for robot-{robot_id}: [{new_goal.x}, {new_goal.y}]")

    def resolve_goal_conflicts(self):
        '''
        Choose another goal if two robots have chosen the same goal
        '''
        conflict = True
        while conflict:
            conflict = False
            for robot_i, goal_i in enumerate(self.final_goal):
                for robot_j, goal_j in enumerate(self.final_goal):
                    if robot_i > robot_j and goal_i == goal_j:
                        self.get_logger().info(f"Same goal for robot-{robot_i} and robot-{robot_j}")
                        new_goal = self.generate_new_goal(goal_i)
                        self.update_goal(robot_i, new_goal)
                        conflict = True
                        
    def resolve_obstacle_conflicts(self):
        '''
        Choose another goal if a robot's goal is inside an obstacle
        '''
        conflict = True
        while conflict:
            conflict = False
            for robot_i, goal_i in enumerate(self.final_goal):
                for obstacle_i in self.obstacles:
                    if int(goal_i.x + SHIFT_MAP) == int(obstacle_i[0]) and int(goal_i.y + SHIFT_MAP) == int(obstacle_i[1]):
                        self.get_logger().info(f"Robot-{robot_i} goal ({goal_i}) is inside obstacle-{obstacle_i}")
                        new_goal = self.generate_new_goal(goal_i)
                        self.update_goal(robot_i, new_goal)
                        conflict = True

    def generate_new_goal(self, old_goal):
        '''
        Generate a new goal close to the old goal (randonly pick a neighboring point)
        '''
        new_goal = Point(x=old_goal.x + random.randint(-1, 1),
                         y=old_goal.y + random.randint(-1, 1),
                         z=0.0)
        return new_goal
                
    def get_data_from_yaml(self):
        '''
        Get the path planning solution from CBS: A sequence of waypoints for the robots to follow in sincrony
        '''
        output_filename = 'scripts/params/cbs_output.yaml'
        filename = self.get_full_filename(output_filename)
        with open(filename, 'r') as file:
            data = yaml.safe_load(file)
        if not data:
            alarm_msg = String()
            alarm_msg.data = 'Solution not found!'
            self.alarm_publisher.publish(alarm_msg)
            self.get_logger().info('Solution not found!')
            time.sleep(2)
            self.first_time_planning = True
            self.generate_new_targets()
            return
        else:
            status = data["status"]
            if status == 0:
                alarm_msg = String()
                alarm_msg.data = 'Solution not found!'
                self.alarm_publisher.publish(alarm_msg)
                self.get_logger().info('Solution not found!')
                time.sleep(2)
                self.first_time_planning = True
                self.generate_new_targets()
                return
            else:
                alarm_msg = String()
                alarm_msg.data = 'Solution found!'
                self.alarm_publisher.publish(alarm_msg)
                self.get_logger().info('Solution found!')

            schedule = data['schedule']
            self.target_waypoints = [[] for _ in range(len(self.robots))]
            self.max_cbs_times = [int for _ in range(len(self.robots))]
            for robot, waypoints in schedule.items():
                robot_index = re.search(r'robot(\d+)', robot).group(1)
                for wp in waypoints:
                    waypoint = wp.items()
                    for item in waypoint:
                        key, value = item                    
                        if key == 'x':
                            x_value = value
                        elif key == 'y':
                            y_value = value
                        elif key == 't':
                            t_value = value
                            self.max_cbs_times[int(robot_index)] = int(t_value)
                    self.target_waypoints[int(robot_index)].append(Point(x=float(x_value), y=float(y_value), z=0.01))
            self.get_logger().info(f'Max number of waypoints to execute: {max(self.max_cbs_times)}')
    
    def get_full_filename(self, param_filename):   
        '''
        Get full filename (workround to get the root of a package) 
        '''
        package_path = packages.get_package_share_directory('planning')
        substring = 'install/planning/share/'
        filename = os.path.join(package_path, param_filename)
        filename = filename.replace(substring, '')
        return filename
    
    def all_positions_received(self):
        '''
        Check if the position of all robots have been received. Avoid errors of computing planning before gettting actual position, calculating with (0,0,0) initial published position
        '''
        if self.first_time_planning == True:
            self.position_received = 0
            for robot_index, _ in enumerate(self.robots):
                current_position = self.positions[robot_index]
                if current_position.x != 0 and current_position.y != 0:
                    self.position_received += 1                    
            if self.position_received == len(self.robots): # All positions received
                self.get_logger().info('All positions received')
                self.first_time_planning = False
                return True
        return False
        
    def verify_initial_robots_positions(self):
        '''
        Check if two robots start in the same position (in relation to the discretization)
        '''
        for robot_i, _ in enumerate(self.robots):
            for robot_j, _ in enumerate(self.robots):
                if robot_i != robot_j:
                    dist = self.get_distance((self.positions[robot_i].x - self.positions[robot_j].x), (self.positions[robot_i].y - self.positions[robot_j].y))
                    if dist < 0.8*(1/DISCRETIZATION):
                        self.get_logger().info('Robots are too close, CBS will not find a solution')
                        
    def call_cbs_planner(self):
        '''
        Compute collision free paths for all robots and read solution from file
        '''
        self.verify_initial_robots_positions()
        self.halt_robots()
        self.get_logger().info(f'Conflict Based Search Planning')
        self.write_data_to_yaml()
        time.sleep(1)
        self.get_logger().info(f'Searching for solution...')
        cbs.main()
        time.sleep(1)
        self.get_data_from_yaml()
        
    def get_distance(self, dx, dy):
        return math.sqrt(dx*dx + dy*dy)

def main(args=None):
    rclpy.init(args=args)
    planning_and_control = Planner()
    rclpy.spin(planning_and_control)
    planning_and_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()