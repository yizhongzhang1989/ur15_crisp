cmake_minimum_required(VERSION 3.8)
project(ur15_dashboard)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/dashboard_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY static/
  DESTINATION share/${PROJECT_NAME}/static
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
