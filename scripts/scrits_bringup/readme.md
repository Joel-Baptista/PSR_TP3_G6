This package contains a launch file that set a different level of intelligence for each robot, this means, each robot will run a different script so that we can compare the results.

Current :
Blue Team -> camera_and_lidar.py

Red team -> oriented_goal_cmd_vel.py

Green team -> camera_and_lidar.py

**Use 4 different terminals:**

Launch arena:

    roslaunch p_g06_bringup th_arenas.launch arena:=1

Launch robots:

    roslaunch p_g06_multi_robot multi_robot.launch

Launch scripts:

    roslaunch scrits_bringup multi_robot_multi_script.launch

Launch referee:

    rosrun th_referee th_referee 
