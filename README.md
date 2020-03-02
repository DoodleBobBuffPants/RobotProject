# RobotProject
MRS Project

## Requirements
* Copy the 'turtlebot3', 'turtlebot3_msgs', and 'turtlebot3\_simulations' folders from 'catkin\_ws/src/' from the original exercises.

* Copy the 'project' folder into 'catkin\_ws/src/exercises' and run the code from there.

## When running the code
Launch gazebo using the environment you wish.

Go to `init_formations.py` and set MAP\_PARAMS to the map you want (e.g SIMPLE\_MAP)

If you want to run RRT, set RUN\_RRT to True.

Predefined paths are stored in the `precomputed_rrt_paths.py` file.

NOTE: If you change the starting positions of the robots in the environment, you need to recompute a new path, then either store it in the precomputed paths file or reset the starting positions.

## Decentralized
* To run the decentralized branch, start any map as usual then launch `formation_move_decentralized.launch` for each robot, supplying `id` as an arg (e.g id:=1)

