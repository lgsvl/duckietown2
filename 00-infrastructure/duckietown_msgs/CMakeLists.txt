cmake_minimum_required(VERSION 3.5)

project(duckietown_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


set(msg_files
    #"msg/Header.msg"
  "msg/CarControl.msg"
  "msg/CoordinationSignal.msg"
  "msg/DuckieSensor.msg"
  "msg/LanePose.msg"
  "msg/LEDDetection.msg"
  "msg/LEDDetectionArray.msg"
  "msg/LEDDetectionDebugInfo.msg"
  "msg/WheelsCmd.msg"
  "msg/WheelsCmdStamped.msg"
  "msg/Pose2DStamped.msg"
  "msg/SignalsDetection.msg"
  "msg/Twist2DStamped.msg"
  "msg/AprilTagDetection.msg"
  "msg/AprilTagDetectionArray.msg"
  "msg/AprilTagsWithInfos.msg"
  "msg/TagInfo.msg"
  "msg/Pixel.msg"
  "msg/Vector2D.msg"
  "msg/Segment.msg"
  "msg/SegmentList.msg"
  "msg/Rect.msg"
  "msg/Rects.msg"
  "msg/SceneSegments.msg"
  "msg/SourceTargetNodes.msg"
  "msg/CoordinationClearance.msg"
  #  "msg/VehicleCorners.msg"
  "msg/VehiclePose.msg"
  "msg/FSMState.msg"
  "msg/BoolStamped.msg"
  "msg/StopLineReading.msg"
  "msg/LEDInterpreter.msg"
  # "msg/AntiInstagramHealth.msg"
  "msg/AntiInstagramTransform.msg"
  "msg/KinematicsWeights.msg"
  "msg/KinematicsParameters.msg"
  # "msg/ThetaDotSample.msg"
  # "msg/Vsample.msg"
  "msg/Trajectory.msg"
  "msg/ObstacleType.msg"
  "msg/ObstacleImageDetection.msg"
  "msg/ObstacleImageDetectionList.msg"
  "msg/ObstacleProjectedDetection.msg"
  "msg/ObstacleProjectedDetectionList.msg"
  "msg/ClassifiedObject.msg"
)

set(srv_files
  "srv/SetValue.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs visualization_msgs
  ADD_LINTER_TESTS
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
