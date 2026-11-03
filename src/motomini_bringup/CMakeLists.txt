cmake_minimum_required(VERSION 3.22)
project(motomini_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/virtual_conveyor.py
  scripts/inform_teach_pendant.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
