Launch gazebo with game arena:

    roslaunch p_g06_bringup th_arenas.launch 

launch one robot with name, color, position and orientation:

    roslaunch p_g06_bringup bringup.launch player_name:=red1 player_color:=Red x_pos:=2 y_pos:=-1 yaw:=1.57

Launch the navigation node for mapping:

    roslaunch p_g06_navigation p_g06_navigation.launch map_file:="yaml file path"
    
Launch 3 robots of each color (blue, red and green):
    
    roslaunch p_g06_multi_robot multi_robot.launch
