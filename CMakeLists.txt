cmake_minimum_required(VERSION 2.8.3)
project(ode_simulator)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
)
find_package(Boost 1.40 COMPONENTS program_options REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ode_simulator
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS} ${roscpp_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ executable

set(CMAKE_C_FLAGS "--std=gnu99 ${CMAKE_C_FLAGS}")
add_executable(symToODE
    src/symToODE.c
    src/Symbolic.c
    )
target_link_libraries(symToODE m)


add_executable(test_quad2dof_node src/test_quad2dof_node.cpp)
target_link_libraries(test_quad2dof_node
    ${catkin_LIBRARIES}
)


add_executable(quad2dof_node src/ode_simulator_node.cpp src/quad2dof/odefun.c src/MatrixLite.c)
target_include_directories(quad2dof_node PRIVATE src/quad2dof)
target_link_libraries(quad2dof_node ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} ${catkin_LIBRARIES})

