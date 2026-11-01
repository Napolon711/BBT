cmake_minimum_required(VERSION 3.8)
project(ros2_libxr)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rm2027_interfaces REQUIRED)

add_library(referee_codec STATIC
  src/shared_topic.cpp
  src/referee_2027.cpp
)
target_include_directories(referee_codec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_options(referee_codec PRIVATE -Wall -Wextra)

add_executable(ros2_libxr_node src/bridge_node.cpp)
target_link_libraries(ros2_libxr_node referee_codec)

ament_target_dependencies(ros2_libxr_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  rm2027_interfaces
)

install(TARGETS ros2_libxr_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/fake_lower_machine.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME bridge_integration_test COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/bridge_integration_test.py $<TARGET_FILE:ros2_libxr_node>)
  set_tests_properties(bridge_integration_test PROPERTIES TIMEOUT 20)
endif()

ament_package()
