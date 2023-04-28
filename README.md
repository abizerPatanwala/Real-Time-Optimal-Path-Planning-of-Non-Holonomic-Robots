# Real-Time-Optimal-Path-Planning-of-Non-Holonomic-Robots
This project focuses on path planning of non-holonomic robots in dynamic environments. For the environment amazon warehouse environment is chosen in gazebo and created a 
2d map of it. For global planners AIT* and BIT* are chosen and compared with RRT. For local planner MPC is used. 

## Running the planner in 2D:
To run the global planners in 2d, download the files from *global planners* folder and run the following commands:
- *python3 batch_informed_trees.py* for BIT*
- *python3 adaptively_informed_trees_star.py* for AIT*
- *python3 rrt.py* for RRT
TO run the MPC in 2d, download the file from folder *local planner* and run *python3 MPC.py*
