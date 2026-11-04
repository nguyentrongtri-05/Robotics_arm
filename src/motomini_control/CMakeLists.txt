cmake_minimum_required(VERSION 3.22)
project(motomini_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY plc DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/virtual_conveyor.py
  scripts/inform_teach_pendant.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

