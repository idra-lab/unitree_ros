cmake_minimum_required(VERSION 3.0.2)
project(unitree_legged_real)

set(CMAKE_BUILD_TYPE "Release")

#cmake_policy(VERSION 3.12)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    unitree_legged_msgs
    sensor_msgs
)

catkin_package(CATKIN_DEPENDS roscpp unitree_legged_msgs)

message("CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()


# Assume the sdk is stored inside the $HOME folder
set(UNITREE_LEGGED_SDK_PATH "$ENV{HOME}/unitree_legged_sdk")
set(UNITREE_LEGGED_SDK_LIB_PATH "${UNITREE_LEGGED_SDK_PATH}/lib/cpp/${ARCH}")


link_directories("${UNITREE_LEGGED_SDK_LIB_PATH}")
set(EXTRA_LIBS -pthread libunitree_legged_sdk.so lcm)
message(STATUS "C++ standard is set to: ${CMAKE_CXX_STANDARD}")

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${UNITREE_LEGGED_SDK_PATH}/include
)

#add_executable(example_walk src/exe/example_walk.cpp)
#target_link_libraries(example_walk ${EXTRA_LIBS} ${catkin_LIBRARIES})
#add_dependencies(example_walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(example_sin_joint_state src/exe/example_sin_joint_state.cpp)
#target_link_libraries(example_sin_joint_state ${EXTRA_LIBS} ${catkin_LIBRARIES})
#add_dependencies(example_sin_joint_state ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_library(${PROJECT_NAME} src/UnitreeUdpInterface.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${EXTRA_LIBS})

add_executable(ros_udp src/exe/ros_udp.cpp)
target_link_libraries(ros_udp ${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(ros_udp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ros_udp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
