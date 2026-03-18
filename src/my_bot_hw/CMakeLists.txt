cmake_minimum_required(VERSION 3.16)
project(my_bot_hw)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(my_bot_hw SHARED
  src/my_bot_hw_interface.cpp
)

target_compile_features(my_bot_hw PUBLIC cxx_std_17)
target_include_directories(my_bot_hw PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/my_bot_hw>
)

ament_target_dependencies(my_bot_hw
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface my_bot_hw.xml)

install(TARGETS my_bot_hw
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/my_bot_hw
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
