# RobotProject
MRS Project

## Requirements
* Copy the 'turtlebot3', 'turtlebot3_msgs', and 'turtlebot3\_simulations' folders from 'catkin\_ws/src/' from the original exercises.

* Copy the 'project' folder into 'catkin\_ws/src/exercises' and run the code from there.

## When running the code
Launch gazebo using the environment you wish.
Go to `init_formations.py` and set MAP_PARAMS to the map you want (e.g SIMPLE_MAP or CORRIDOR_MAP)

If you want to run RRT, set the parameter RUN_RRT to True. This will display the path and print out the points.

Predefined paths are stored in the `precomputed_rrt_paths.py` file.

NOTE: if you change the starting position of the robots in the environment, you need to precompute a new path (either store it in the precomputed paths file or put the starting position back after).
