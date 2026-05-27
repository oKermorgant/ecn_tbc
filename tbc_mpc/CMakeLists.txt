cmake_minimum_required(VERSION 3.5)
project(tbc_mpc)

find_package(ament_cmake_auto REQUIRED)

install(DIRECTORY launch zoe DESTINATION share/${PROJECT_NAME})

file(GLOB scripts scripts/*)
install(PROGRAMS ${scripts} DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
