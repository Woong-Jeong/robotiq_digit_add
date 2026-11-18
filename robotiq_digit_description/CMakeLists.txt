cmake_minimum_required(VERSION 3.8)
project(robotiq_digit_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes launch rviz third_party
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md validation.json assembly_preview.png DESTINATION share/${PROJECT_NAME})
ament_package()
