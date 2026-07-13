cmake_minimum_required(VERSION 3.8)
project(my_bot_nav)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config docs launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/keepout_mask_reloader.py
  scripts/neupan_metrics.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
