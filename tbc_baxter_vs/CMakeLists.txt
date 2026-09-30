cmake_minimum_required(VERSION 3.15)
project(tbc_baxter_vs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(VISP REQUIRED COMPONENTS core visual_features)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

# ROS deps are resolved here
ament_auto_add_library(baxter_arm SHARED src/baxter_arm.cpp)

# with ViSP as external lib
target_include_directories(baxter_arm PUBLIC include ${VISP_INCLUDE_DIRS})
target_link_libraries(baxter_arm ${VISP_LIBRARIES})

message(STATUS "VISP includes: ${VISP_INCLUDE_DIRS}")
message(STATUS "visp targets: ${VISP_LIBRARIES}")

ament_auto_add_executable(baxter_vs main.cpp)
target_link_libraries(baxter_vs baxter_arm)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES src/ball_motion.py DESTINATION lib/${PROJECT_NAME})

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
