cmake_minimum_required(VERSION 3.8)
project(my_bot_nav)

find_package(ament_cmake REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# 全局规划器外壳：包住 SmacPlanner2D，等价路线不来回换
add_library(sticky_planner SHARED src/sticky_planner.cpp)
ament_target_dependencies(sticky_planner
  rclcpp rclcpp_lifecycle pluginlib nav2_core nav2_costmap_2d nav2_util nav_msgs geometry_msgs tf2)
pluginlib_export_plugin_description_file(nav2_core sticky_planner_plugin.xml)
install(TARGETS sticky_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY config docs launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/keepout_mask_reloader.py
  scripts/keepout_files.py
  scripts/loc_error_test.py
  scripts/map_eval.py
  scripts/map_group.sh
  scripts/odom_tf_relay.py
  scripts/record_bag.sh
  scripts/truth_render.py
  scripts/truth_tf_relay.py
  scripts/neupan_metrics.py
  scripts/nav_route_test.py
  scripts/path_center_check.py
  scripts/plan_curvature.py
  scripts/plan_sweep.py
  scripts/neupan_sweep.py
  scripts/nav_lap_report.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
