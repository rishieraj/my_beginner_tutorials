# ROS2 Beginner Tutorials

This branch contains a C++ package that can run a ROS2 publisher and subscriber communicating a custom message.

**Author:** Rishie Raj (120425554)

### Dependencies
It is assumed that the user has ROS2 Humble installed in their local system and sourced in the environment. The code dependes on the following ROS2 packages:  
 - `ament_cmake`: Required for ROS2 build system integration.
 - `rclcpp`: Used for creating nodes and communication between them.
 - `std_msgs`: Provides standard message types used in ROS2.

### Building the Code

```bash
source /opt/ros/humble/setup.bash
# Make your ros2 workspace
mkdir -p ~/ros2_ws/src
# Go to the source directory of your ros2 workspace
cd ~/ros2_ws/src
#Clone the repository
git clone git@github.com:rishieraj/my_beginner_tutorials.git
#Go back to the ws directory
cd ~/ros2_ws
# Install rosdep dependencies before building the package
rosdep install -i --from-path src --rosdistro humble -y
# Build the package using colcon build
colcon build --packages-select beginner_tutorials
# After successfull build source the package
source install/setup.bash

# Run the publisher in terminal#1
ros2 run beginner_tutorials talker
# Run the subscriber in terminal#2 (Split the terminal and source ROS2 and the workspace setup.bash)
ros2 run beginner_tutorials listener 
```
### Calling the Service

A service has been added to the talker node that can change the output string of the talker based on user input. The service can be called using the following command:

```bash
ros2 service call /change_string beginner_tutorials/srv/ChangeStr "{new_string: User Input}"
```