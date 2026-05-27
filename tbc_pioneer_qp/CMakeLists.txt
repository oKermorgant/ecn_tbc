cmake_minimum_required(VERSION 3.5)
project(tbc_pioneer_qp)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_environment_hooks("hooks/resource_paths.dsv.in")

set(CMAKE_BUILD_TYPE Debug)

find_package(VISP REQUIRED COMPONENTS core visual_features)

# Gazebo version, use env variable or ROS 2 official pairing for LTS
# https://gazebosim.org/docs/latest/ros_installation/
if(DEFINED ENV{GZ_VERSION})
  set(GZ_VERSION "$ENV{GZ_VERSION}")
elseif(DEFINED ENV{IGN_VERSION})
  set(GZ_VERSION "$ENV{IGN_VERSION}")
elseif("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "lyrical")
  set(GZ_VERSION "jetty")
elseif("$ENV{ROS_DISTRO}" STRGREATER_EQUAL "jazzy")
  set(GZ_VERSION "harmonic")
else()
  set(GZ_VERSION "fortress")
endif()

message(STATUS "Compiling against Gazebo ${GZ_VERSION}")
set(GZ_TARGET_PREFIX gz)

# check GZ version
if("${GZ_VERSION}" STRGREATER_EQUAL "jetty")
  set(GZ_MSGS msgs)
  set(GZ_TRANSPORT transport)

elseif("${GZ_VERSION}" STREQUAL "harmonic")
  set(GZ_MSGS msgs10)
  set(GZ_TRANSPORT transport13)

elseif("${GZ_VERSION}" STREQUAL "garden")
  set(GZ_MSGS msgs9)
  set(GZ_TRANSPORT transport12)

else()
  set(GZ_TARGET_PREFIX ignition)
  set(GZ_MSGS msgs8)
  set(GZ_TRANSPORT transport11)
  add_definitions(-DFOR_IGNITION_FORTRESS)
endif()

find_package(${GZ_TARGET_PREFIX}-${GZ_MSGS})
find_package(${GZ_TARGET_PREFIX}-${GZ_TRANSPORT})

add_definitions(-DGZ_NS=${GZ_TARGET_PREFIX})
  
set(CMAKE_CXX_STANDARD 17)

include_directories(include ${VISP_INCLUDE_DIRS})
ament_auto_add_library(tbc_pioneer_qp SHARED src/pioneer_cam.cpp)
target_link_libraries(tbc_pioneer_qp 
    ${VISP_LIBRARIES}
    ${GZ_TARGET_PREFIX}-${GZ_MSGS}::core
    ${GZ_TARGET_PREFIX}-${GZ_TRANSPORT}::core
)

ament_auto_add_executable(pioneer src/main.cpp)
target_link_libraries(pioneer tbc_pioneer_qp)

install(DIRECTORY launch models DESTINATION share/${PROJECT_NAME})

install(TARGETS pioneer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
