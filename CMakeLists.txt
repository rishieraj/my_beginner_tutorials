cmake_minimum_required(VERSION 3.8)
project(beginner_tutorials)

# Set the C++ standard to C++17 and require it for all targets
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Create a symlink to the compile_commands.json file in the source directory
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # Enable compile_commands.json
execute_process(COMMAND cmake -E create_symlink
                "${PROJECT_BINARY_DIR}/compile_commands.json"
                "${PROJECT_SOURCE_DIR}/compile_commands.json")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(catch_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosbag2 REQUIRED)

# Generate interfaces in the 'srv' directory
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ChangeStr.srv"
  DEPENDENCIES std_msgs
)

# Adding the executables
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker 
  rclcpp
  std_msgs
  rclcpp
  rosidl_default_generators
  geometry_msgs
  rclcpp
  tf2_ros)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# Export runtime dependencies
ament_export_dependencies(rosidl_default_runtime)

# Get the typesupport for the C++ interfaces
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Link the executables to the typesupport
target_link_libraries(talker "${cpp_typesupport_target}")

# Install the executables
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

# Install the launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # disable these built-in tests
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  # build the integration test
  ###########################################################################
  # 1.) BUILD AND INSTALL THE INTEGRATION TEST NODE (aka Node that performs the test)  ##
  ###########################################################################
  add_executable(integration_test_node
    test/integration_test_node.cpp
  )

  target_link_libraries(integration_test_node
    catch_ros2::catch_ros2_with_node_main
    "${cpp_typesupport_target}"
  )

  ament_target_dependencies(integration_test_node
    rclcpp
    std_msgs
    rosidl_default_runtime
    rosidl_default_generators
    geometry_msgs
  )

  install(TARGETS
    integration_test_node
    DESTINATION lib/${PROJECT_NAME}
  )

  ######################################
  ## 2.) ADD INTEGRATION TEST SUPPORT ##
  ######################################
  catch_ros2_add_integration_test (Catch2IntegrationTest
    LAUNCH_FILE integration_test.launch.yaml
  )
endif()


ament_package()
