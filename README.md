# Real-Time-Optimal-Path-Planning-of-Non-Holonomic-Robots
This project focuses on path planning of non-holonomic robots in dynamic environments. For the environment amazon warehouse environment is chosen in gazebo and created a 
2d map of it. For global planners **AIT*** and **BIT*** are chosen and compared with **RRT**. For local planner **MPC** is used and compared with **APF**. Gazebo simulation is done with ROS2 Humble.


## Global Planners:
### AIT*
![AIT* output in 2D environment](/outputs/ait*.gif)
### BIT*
![BIT* output in 2D environment](/outputs/bit*.gif)

## Traversal with obstacles
### 2D toy problem
![trajectory traversal output in 2D environment](/outputs/mpc.gif)
### Gazebo simulation
![trajectory traversal output in warehouse environment](/outputs/gazebo.gif)

## Running the planner in 2D:
To run the global planners in 2d, download the files from *global planners* folder and run the following commands:
- *python3 batch_informed_trees.py* for BIT*
- *python3 adaptively_informed_trees_star.py* for AIT*
- *python3 rrt.py* for RRT

To run the MPC in 2d, download the file from folder *local planner* and run *python3 MPC.py*

## Running the planners in gazebo:
To run MPC in gazebo, download the *src* folder. Go to the directory where *src* is downloaded and run *colcon build* in terminal. After the package is build,
run the following commands from the root folder(where src is placed) for completing the setup. These commands should be run everytime before running the simulation.
- *. install/local_setup.bash*
- *export TURTLEBOT3_MODEL=burger*
- *export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:\`ros2 pkg \
prefix turtlebot3_gazebo \
\`/share/turtlebot3_gazebo/models/*

After completing the setup run the following command for spawning the robot and world in gazebo, *ros2 launch gazbeosim world_with_robot_new.launch.py*. Then run the 
following command for runing MPC in gazebo *ros2 run planner MPC*.
