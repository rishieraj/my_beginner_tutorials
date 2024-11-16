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

### Launching the Pub and Sub Nodes

In order to initiate the publisher and subscriber node together using a launch file, the following command can be run on the terminal after sourcing the package. It accepts an argument `publish_frequency` whose value can be initialized by the user.

```bash
ros2 launch beginner_tutorials launch.py publish_frequency:=1000
```

### Launching Pub along with Rosbag

In order to launch the nodes along with `rosbag`, the following command needs to be run with the parameter flag
```bash
ros2 launch beginner_tutorials launch.py publish_frequency:=1000 enable_recording:=true
```

### Calling the Service

A service has been added to the talker node that can change the output string of the talker based on user input. The service can be called using the following command:

```bash
ros2 service call /change_string beginner_tutorials/srv/ChangeStr "{new_string: User Input}"
```

### Replaying Rosbag Topic Info
In order for the Rosbag talker recodings to be replayed so that the listener can pick them up, the following commands can be run in sequence.  
Open a fresh terminal and source the overlay as below:
```bash
source /opt/ros/humble/setup.bash
# Go back to the ws directory
cd ~/ros2_ws
# After successfull build source the package
source install/setup.bash
# Run the subscriber node
ros2 run beginner_tutorials listener
```
Then in another terminal, run the following
```bash
# Source the package
source install/setup.bash
# Play the rosbag recording
ros2 bag play ./src/beginner_tutorials/results/rosbag2_2024_11_15-20_12_53
```
On returning to the listener terminal, it can be seen that the listener picks up the topic recordings of the talker.

### Running Integration Tests using Catch2
In order to check the results of the integration test, the following commands need to be run in sequence. In a new terminal
```bash
# Source the package
source install/setup.bash
# Run the test
colcon test --packages-select beginner_tutorials
# Display the output
cat log/latest_test/integration_test/stdout_stderr.log
```