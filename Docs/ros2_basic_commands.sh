box_bot_gazebo

ros2 pkg create box_bot_gazebo --build-type ament_cmake --dependencies ament_cmake rclcpp rclpy

ros2 pkg create box_bot_description --build-type ament_cmake --dependencies ament_cmake urdf


colcon build --symlink-install --packages-ignore dolly_ignition

# Initial Setup ROS and colcon
# https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html#prerequisites

source /opt/ros/foxy/setup.bash
# Only compile certian packages
colcon build --symlink-install --packages-select box_bot_gazebo box_bot_description
# Compile everything if you want
colcon build --symlink-install
source install/setup.bash

# Install Dependencies
# https://docs.ros.org/en/dashing/Installation/Linux-Development-Setup.html#install-dependencies-using-rosdep
sudo rosdep init
rosdep update
# rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"
rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 rti-connext-dds-5.3.1 urdfdom_headers"


# Launch single liner
ros2 launch box_bot_gazebo box_bot_launch.py
ros2 launch box_bot_gazebo multi_box_bot_launch.py
# If you want separted
ros2 launch box_bot_gazebo start_world_launch.py
ros2 launch box_bot_description start_world_launch.py

ros2 topic list

ros2 topic pub /box_bot/cmd_vel 


ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/box_bot/cmd_vel

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/box_bot2/cmd_vel
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/box_bot2/cmd_vel


ros2 run tf2_tools view_frames.py

# Snapcraft
# For ubuntu 20
sudo apt update
sudo apt install snapd
sudo snap install core20

sudo snap install snapcraft --classic
>> snapcraft 4.5.1 from Canonical✓ installed
# Say yes to multi, and we enable experimental because otherwise it wont be able to add extensions: [ros2-foxy] in the app section
snapcraft --enable-experimental-extensions

# Install the snap once its done
sudo snap install box-bot-ros2_0.1_amd64.snap --devmode --dangerous


sudo snap remove box-bot-ros2

###########


This part is missing libraries that cannot be satisfied with any available stage-packages known to snapcraft:
- libnddsc.so
- libnddscore.so
- libnddscpp.so
- librosidl_typesupport_connext_c.so
- librosidl_typesupport_connext_cpp.so
- librticonnextmsgcpp.so
- usr/lib/x86_64-linux-gnu/libpsm_infinipath.so.1
These dependencies can be satisfied via additional parts or content sharing. Consider validating configured filesets if this dependency was built.


