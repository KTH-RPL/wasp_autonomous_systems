cmake_minimum_required(VERSION 3.5)
project(webots_ros2_msgs)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(APPLE)
  # See src/wasp_as_interfaces/CMakeLists.txt for why: upstream
  # rosidl_typesupport_introspection_c-generated code trips
  # -Wint-conversion/-Wincompatible-function-pointer-types, which Xcode's
  # Clang 17 defaults to errors in C mode. This package also generates
  # interfaces via rosidl_generate_interfaces, so it hits the same issue.
  add_compile_options(-Wno-error=int-conversion -Wno-error=incompatible-function-pointer-types)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

set(msg_files
  "msg/BoolStamped.msg"
  "msg/FloatStamped.msg"
  "msg/StringStamped.msg"
  "msg/CameraRecognitionObject.msg"
  "msg/CameraRecognitionObjects.msg"
  "msg/UrdfRobot.msg"
  "msg/PenInkProperties.msg"
)
set(srv_files
  "srv/EmitterSendString.srv"
  "srv/GetBool.srv"
  "srv/SetString.srv"
  "srv/SpawnNodeFromString.srv"
  "srv/SpawnUrdfRobot.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files} ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs vision_msgs
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
