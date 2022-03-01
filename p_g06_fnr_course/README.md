**Launch world**

    roslaunch p_g06_bringup fnr_course.launch 

**Launch robot on start position for teleop**

    roslaunch p_g06_bringup fnr_bringup.launch

**Launch teleop**

    roslaunch p_g06_bringup p_g06_teleop.launch 

**Follow line:##################################################**

Video: https://www.youtube.com/watch?v=AuPEjX6V6Ew

**Launch fnr without obstacles**
    
    roslaunch p_g06_bringup fnr_no_obs.launch 

**Launch robot on start position for follow line**

    roslaunch p_g06_bringup fnr_no_obs_bringup.launch

**Launch "autonomous driving"**

    rosrun p_g06_fnr_course fnr_auto_drive.py 