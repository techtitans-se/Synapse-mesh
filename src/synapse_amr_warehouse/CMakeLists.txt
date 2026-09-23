cmake_minimum_required(VERSION 3.8)
project(synapse_amr_warehouse)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  worlds
  models
  urdf
  maps
  rviz
  data
  DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
  scripts/amr_rule_engine.py
  scripts/teleop_keyboard.py
  scripts/amr_task_executor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
