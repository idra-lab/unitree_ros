cmake_minimum_required(VERSION 3.0.2)
project(unitree_legged_real)

add_compile_options(-std=c++11)

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

set(EXTRA_LIBS -pthread libunitree_legged_sdk.so lcm)

set(CMAKE_CXX_FLAGS "-O3 -fPIC")

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    /home/unitree/unitree_legged_sdk/include
)

#add_executable(example_walk src/exe/example_walk.cpp)
#target_link_libraries(example_walk ${EXTRA_LIBS} ${catkin_LIBRARIES})
#add_dependencies(example_walk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_executable(example_position src/exe/example_position.cpp)
#target_link_libraries(example_position ${EXTRA_LIBS} ${catkin_LIBRARIES})
#add_dependencies(example_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#link_directories(/home/mcamurri/unitree_legged_sdk/lib)
link_directories(/home/mcamurri/catkin_ws/src/unitree_legged_sdk/lib)

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
