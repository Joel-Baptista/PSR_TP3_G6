Launch gazebo with game arena (alterado por Gonçalo):

    **Lança arena 1 por defeito:**

        roslaunch p_g06_bringup th_arenas.launch 

    **Lançar arena 2:**

        roslaunch p_g06_bringup th_arenas.launch arena:=2

    **Lançar arena 3:**

        roslaunch p_g06_bringup th_arenas.launch arena:=3


launch one robot with name, color, position and orientation:

    roslaunch p_g06_bringup bringup.launch player_name:=red1 player_color:=Red x_pos:=2 y_pos:=-1 yaw:=1.57

Launch the navigation node for mapping:

    roslaunch p_g06_navigation p_g06_navigation.launch map_file:="yaml file path"
    
Launch 3 robots of each color (blue, red and green):
    
    roslaunch p_g06_multi_robot multi_robot.launch
