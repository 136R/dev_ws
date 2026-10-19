cmake_minimum_required(VERSION 3.8)
project(my_bot_nav)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config docs launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/keepout_mask_reloader.py
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
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
