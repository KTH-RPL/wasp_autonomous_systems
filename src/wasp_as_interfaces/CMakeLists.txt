cmake_minimum_required(VERSION 3.10)
project(wasp_as_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(APPLE)
  # Upstream ROS2 code generator bug (rosidl_typesupport_introspection_c),
  # not ours: its generated *__type_support.c files assign NULL (a void*,
  # via Xcode's `#define NULL ((void*)0)`) to uint32_t "default value"
  # fields, and pass init functions with a mismatched signature. Older/
  # looser Clang only warned about this; Xcode's bundled Clang 17 (used for
  # this LOCAL colcon build, distinct from the newer conda-provided Clang
  # used by pixi-build-ros's isolated builds - see webots_ros2_driver/
  # CMakeLists.txt) defaults -Wint-conversion and
  # -Wincompatible-function-pointer-types to errors in C mode. Demote them
  # back to warnings rather than patching generated code we don't own.
  add_compile_options(-Wno-error=int-conversion -Wno-error=incompatible-function-pointer-types)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Collision.msg"
  "msg/DutyCycles.msg"
  "msg/Encoders.msg"
  "msg/Thrust.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

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
