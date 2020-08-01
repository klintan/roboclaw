cmake_minimum_required(VERSION 3.5.0)

project(roboclaw)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
# https://github.com/RoverRobotics-forks/serial-ros2
find_package(serial REQUIRED)


include_directories(include)

add_library(roboclaw SHARED
  src/roboclaw/roboclaw.cpp)

target_compile_definitions(roboclaw
PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(roboclaw
  "serial"
)

ament_export_targets(export_roboclaw HAS_LIBRARY_TARGET)
ament_export_dependencies(serial)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS roboclaw
  EXPORT export_roboclaw
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
