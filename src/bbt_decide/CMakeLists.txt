cmake_minimum_required(VERSION 3.16)
project(bbt_decide)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 导出 compile_commands.json（build/<pkg>/ 下）：编辑器/clangd 靠它解析头文件路径，
# 没有它 VSCode 里所有 #include 都会标红。仓库根的软链接指向它。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

# 纯逻辑库：决策树 + 过洞状态机 + 配置加载，不依赖 ROS，测试直接链接
add_library(bbt_decide_core
  src/planner.cpp
  src/config.cpp)
target_include_directories(bbt_decide_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(bbt_decide_core PUBLIC yaml-cpp)

# 行为树节点：编排层，逻辑全在 bbt_decide_core 的纯函数里
add_library(bbt_decide_bt src/bt.cpp)
target_link_libraries(bbt_decide_bt PUBLIC bbt_decide_core behaviortree_cpp::behaviortree_cpp)

# ROS 适配层：抽象 Ports ↔ 真实话题（本包唯一数据进出口）
add_library(bbt_decide_ros src/ros_ports.cpp)
ament_target_dependencies(bbt_decide_ros rclcpp std_msgs geometry_msgs nav_msgs)
target_link_libraries(bbt_decide_ros bbt_decide_core)

# 决策节点
add_executable(bbt_decide_node src/main.cpp)
ament_target_dependencies(bbt_decide_node rclcpp std_msgs ament_index_cpp)
target_link_libraries(bbt_decide_node bbt_decide_core bbt_decide_bt bbt_decide_ros)

install(TARGETS bbt_decide_core bbt_decide_bt bbt_decide_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS bbt_decide_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY trees config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  # 纯逻辑测试：不需要 ROS 环境，直接跑（走廊洞/换点/多洞/死亡回血全在里面）
  add_executable(bbt_decide_tests test/planner_test.cpp)
  target_link_libraries(bbt_decide_tests bbt_decide_core)
  add_test(NAME bbt_decide_planner_test COMMAND bbt_decide_tests)

  # 行为树测试：用树驱动同一套走廊流程，证明"跑树"与"直接调纯函数"行为一致
  add_executable(bbt_decide_bt_tests test/bt_test.cpp)
  target_compile_definitions(bbt_decide_bt_tests PRIVATE SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(bbt_decide_bt_tests bbt_decide_bt behaviortree_cpp::behaviortree_cpp)
  add_test(NAME bbt_decide_bt_test COMMAND bbt_decide_bt_tests)
endif()

ament_package()
