cmake_minimum_required(VERSION 3.8)
project(tmcl_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

############################################

set(msg_files
  "msg/TmcInfo.msg"
  "msg/TmcParam.msg"
)

set(srv_files
  "srv/TmcCustomCmd.srv"
  "srv/TmcGapAll.srv"
  "srv/TmcGgpAll.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

############################################

add_library(socket_can_wrapper SHARED
  src/socket_can_wrapper.cpp)

# target_link_libraries(socket_can_wrapper ${rclcpp_LIBRARIES})
target_include_directories(socket_can_wrapper PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)
ament_target_dependencies(socket_can_wrapper rclcpp)
ament_export_targets(socket_can_wrapperTargets HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS socket_can_wrapper
  EXPORT socket_can_wrapperTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include)

############################################

add_library(tmcl_interpreter SHARED
  src/tmcl_interpreter.cpp)

target_link_libraries(tmcl_interpreter socket_can_wrapper)
target_include_directories(tmcl_interpreter PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)

ament_target_dependencies(tmcl_interpreter rclcpp)
ament_export_targets(tmcl_interpreterTargets HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS tmcl_interpreter
  EXPORT tmcl_interpreterTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include)

############################################

add_library(tmcl_motor SHARED
  src/tmcl_motor.cpp)


target_link_libraries(tmcl_motor tmcl_interpreter "${cpp_typesupport_target}") 

target_include_directories(tmcl_motor PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)

ament_target_dependencies(tmcl_motor rclcpp std_msgs geometry_msgs)
ament_export_targets(tmcl_motorTargets HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS tmcl_motor
  EXPORT tmcl_motorTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include)

############################################

add_library(tmcl_bldc_motor SHARED
  src/tmcl_bldc_motor.cpp)


target_link_libraries(tmcl_bldc_motor tmcl_motor tmcl_interpreter "${cpp_typesupport_target}") 

target_include_directories(tmcl_bldc_motor PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)

ament_target_dependencies(tmcl_bldc_motor rclcpp std_msgs geometry_msgs)
ament_export_targets(tmcl_bldc_motorTargets HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS tmcl_bldc_motor
  EXPORT tmcl_bldc_motorTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include)

############################################

add_library(tmcl_stepper_motor SHARED
  src/tmcl_stepper_motor.cpp)


target_link_libraries(tmcl_stepper_motor tmcl_motor tmcl_interpreter "${cpp_typesupport_target}") 

target_include_directories(tmcl_stepper_motor PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)

ament_target_dependencies(tmcl_stepper_motor rclcpp std_msgs geometry_msgs)
ament_export_targets(tmcl_stepper_motorTargets HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS tmcl_stepper_motor
  EXPORT tmcl_stepper_motorTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include)

############################################


add_library(tmcl_ros2_lib SHARED
  src/tmcl_ros2.cpp)

target_link_libraries(tmcl_ros2_lib tmcl_motor tmcl_bldc_motor tmcl_stepper_motor "${cpp_typesupport_target}")
target_include_directories(tmcl_ros2_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${rclcpp_INCLUDE_DIRS}
)

ament_target_dependencies(tmcl_ros2_lib rclcpp std_msgs geometry_msgs std_srvs)
ament_export_targets(tmcl_ros2libTargets HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS tmcl_ros2_lib
  EXPORT tmcl_ros2libTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include)

############################################

add_executable(tmcl_ros2_node src/tmcl_ros2_node.cpp)
target_link_libraries(tmcl_ros2_node tmcl_ros2_lib)
ament_target_dependencies(tmcl_ros2_node rclcpp)

install(
  TARGETS tmcl_ros2_node
  DESTINATION lib/${PROJECT_NAME}
)

############################################

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

############################################

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
