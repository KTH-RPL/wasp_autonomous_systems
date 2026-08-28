cmake_minimum_required(VERSION 3.10)
project(wasp_autonomous_systems_interfaces)

if(APPLE)
  # See src/wasp_as_interfaces/CMakeLists.txt for why: upstream
  # rosidl_typesupport_introspection_c-generated code trips
  # -Wint-conversion/-Wincompatible-function-pointer-types, which Xcode's
  # Clang 17 defaults to errors in C mode. This package also generates
  # interfaces via rosidl_generate_interfaces, so it hits the same issue.
  add_compile_options(-Wno-error=int-conversion -Wno-error=incompatible-function-pointer-types)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DutyCycles.msg"
  "msg/Encoders.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
