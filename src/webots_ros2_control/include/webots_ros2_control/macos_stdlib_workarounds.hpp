// macOS-only (see CMakeLists.txt's `if(APPLE)` block that force-includes this
// file). Upstream ROS2 headers (realtime_tools, rosidl_runtime_cpp) rely on
// <locale>/<exception> being pulled in transitively by something else they
// include - true on older/looser libc++, not on this build's stricter one.
// Force-including both here, once, ahead of everything else, papers over
// those missing includes without touching vendored/upstream header content.
#include <locale>
#include <exception>
