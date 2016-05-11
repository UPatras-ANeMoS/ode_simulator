#!/bin/bash

cd data/$1
maxima --batch-string="batchload(\"../../maxima/cog_v2.wxm\")$" &&
cd ../.. &&
mkdir src/$1 &&
rosrun ode_simulator symToODE data/$1 src/$1 &&
echo "" >> CMakeLists.txt &&
echo "add_executable("$1"_node src/ode_simulator_node.cpp src/$1/odefun.c src/MatrixLite.c)" >> CMakeLists.txt &&
echo "target_include_directories("$1"_node PRIVATE src/$1)" >> CMakeLists.txt &&
echo "target_link_libraries("$1"_node \${Boost_LIBRARIES} \${EIGEN3_LIBRARIES} \${catkin_LIBRARIES})" >> CMakeLists.txt &&
echo "" >> CMakeLists.txt
