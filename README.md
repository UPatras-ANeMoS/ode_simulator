ODE Simulator
=============

## Installation Instructions

1. If not already, install  and initialize ROS indigo desktop full (http://wiki.ros.org/indigo/Installation/Ubuntu).
2. Clone ode_simulator in your ROS worspace src folder.
3. Install Maxima, which is a computer algebra system:

		$ sudo apt-get install maxima


## Basic Usage

1. If you haven't done this before, build your workspace:

 		$ cd ~/catkin_ws/
		$ catkin build
		
2. Create a file with the parameters of the UAV, seperated by commas:

		mass,arm's length,g,Ixx,Iyy,Izz
		
	and save it as P_uav.txt

3. Create a file with the parameters of the arm, one line per dof, seperated by commas:

		q,d,r,a,m,lcx,lcy,lcz,Ixx,Iyy,Izz,Ixy,Ixz,Iyz	
	as D–H parameters(4 params),mass,center of mass(3 params),inertia matrix(6 params) respectively,
	and save it as P_arm.txt

4. Place a folder containing the info files for your system in `ode_simulator/data/`.
From `ode_simulator/` call

		$ ./dataToFun.sh name

where `name` is the previous folder name.

On success `src/name` will be created with an ode function for your system.
At the same time a target `name_node` will be added in `CMakeLists.txt`.
This is the simulator node for your system.

Moreover, four additional files will be created in the same folder.

* `D.txt` : The matrix `D` in the form:
```
                [d11, d12, ...
                 d21, d22, ...
                 ..., ..., ...]
```

* `W.txt` : The vector `W` in the form:
```
                [w1
                 w2
                ...]
```


* `X.txt` : The description of the state vector, as the states are used in `D`, `W`.
            eg.
```
                [x,vx, y,vy, z,vz, phi,wx, theta,wy, psi,wz, q1,qd1, q2,qd2, ...]
```

* `U.txt` : The description of the inputs vector, as the inputs are used in `D`, `W`.
            eg.
```
                [ U1, U2, U3, U4, tau1, tau2, ... ]
```

### name_node

#### Parameters:
* `_u` : The inputs topic (`std_msgs::Float64MultiArray`)
* `_x` : The the correction topic (optional) (`std_msgs::Float64MultiArray`)

#### Published Topics:
* `x`  : The state vector (`std_msgs::Float64MultiArray`)


## quad2dof Tester

Calling

	$ roslaunch ode_simulator test_quad2dof.launch

will run the `quad2dof_node` and `test_quad2dof_node`, which implements a hovering controller.
Also it lauches `rqt_plot` presenting the quad's x,y,z coordinates.




