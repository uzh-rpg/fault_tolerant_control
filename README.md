

# Summary
This repository contains the flight controller, and the vision-based state estimator for a quadrotor subjected to complete failure of a single rotor. The program runs in a ROS environment. 

<p align="center">
  <img src="./img/ftc.gif" alt="ftc">
</p>
<!--A general structure is given as follows  -->

## Citing
If you use this work in an academic context, please cite the following RA-L publication:

S. Sun, G. Cioffi, C. de Visser and D. Scaramuzza,
"**Autonomous Quadrotor Flight despite Rotor Failure with Onboard Vision Sensors: Frames vs. Events**,"
IEEE Robot. Autom. Lett. (RA-L). 2021.

    @ARTICLE{SunAutonomous2021,
	  author={S. {Sun} and G. {Cioffi} and C. {de Visser} and D. {Scaramuzza}},
	  journal={IEEE Robotics and Automation Letters}, 
	  title={Autonomous Quadrotor Flight despite Rotor Failure with Onboard Vision Sensors: Frames vs. Events}, 
	  year={2021},
	  volume={},
	  number={},
	  pages={1-1},
	  doi={10.1109/LRA.2020.3048875}}
# Structure

#### \vio_estimator
Vision-based pose estimator using a standard camera. 
#### \ftc_estimator
Rotation-corrected Complementary Filter and Extended Kalman Filter providing full pose and velocity estimates for the fault-tolerant flight controller. 
#### \ftc_ctrl
Fault-tolerant flight controller that generates thrust command of individual rotors, using a Nonlinear Dynamic Inversion approach.

# Installation
This installation guide was tested with Ubuntu 18.04

Requirements:
- CMake >= 3.0
- ROS (>= Kinetic) (see [installation guide](http://wiki.ros.org/ROS/Installation)).

You also need to install the following packages for [Ceres Solver](http://ceres-solver.org/):

    sudo apt install liblapack-dev libblas-dev

Create  a ROS workspace

    mkdir -p ~/catkin_ws/src && cd ~/catkin_ws
    catkin init
Configure the ROS environment

    catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release

Clone the repository and dependencies

    cd src/
    git clone https://github.com/uzh-rpg/fault_tolerant_control.git
    vcs-import < fault_tolerant_control/dependencies.yaml
   
Build the workspace

	cd ~/catkin_ws && catkin build
#### Arm64
If you run the program in arm64 architecture, please add 

	set(ENV{ARM_ARCHITECTURE} aarch64)

in /vio_estimator/common/ze_common/CMakeLists.txt and /vio_estimator/common/ze_cmake/cmake/modules/ze_setup.cmake

# Test the vision-based state esimtator
You can test the vision-based state estimator by running this [**rosbag**](https://seafile.ifi.uzh.ch/d/44ac95d256124af287a3/) recorded from real-flights using a standard camera ([mvBLuefox](https://www.matrix-vision.com/USB2.0-industrial-camera-mvbluefox.html)). 

	roslaunch ze_vio_ceres live_Bluefox.launch
	rosbag play <directory_of_the_bag>/data_bluefox.bag
#### PS: Due to copyright reasons, we cannot release the event-based estimator until 2022. 
	
# Simulation
You can test the flight controller in [RotorS](https://github.com/ethz-asl/rotors_simulator), a MAV gazebo simulator. First of all, please install [Gazebo](http://gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros). 

Then run the following launch file 

	roslaunch ftc_ctrl testSim.launch

The drone is ready to fly in the simulator. We provide some simple commands in the /scripts folder 
	
	roscd ftc_ctrl && cd scripts
To take off, please type

	./start_rotors.sh hummingbird

Then you can switch one motor off by running the script

	./stop_rotor.sh hummingbird
You may decide which rotor to  be turned off in [simulation.yaml](https://github.com/uzh-rpg/fault_tolerant_control/blob/master/ftc_ctrl/parameters/simulation.yaml).

To control the quadrotor to a way point, e.g., xyz = [1.0 2.0 3.0], use the script

	./reference.sh 1.0 2.0 3.0 hummingbird
 
# Real flight test
You may reproduce our experiments in real flights, if you have 
- A mvBluefox camera and it's [ROS driver](https://github.com/KumarRobotics/bluefox2).
- A [Terarranger-One](https://www.terabee.com/shop/lidar-tof-range-finders/teraranger-one/) range sensor and it's [ROS_driver](http://wiki.ros.org/teraranger).
- A quadrotor that you can control the thrust of each individual rotor.
- A Joystick.
- Safety nets ;)

To start the flight controller, please run
	
	roslaunch ftc_ctrl test.launch

The controller will publish rotor thrust command in a rostopic 

	/control_command/rotor_thrust 
You can feed these command to your low-level motor controller.

### Add the vision-based estimator into the loop
Once the drone starts hovering before one motor is switched off, you can start the state estimator by running the following launch files:

	roslaunch ze_vio_ceres live_Bluefox.launch

The estimated state will be published in the ROS topic

	/state_est
which is used by the flight controller.

