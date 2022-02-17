#!/usr/bin/env python3

import argparse

import rospy
from std_msgs.msg import String
from p_g06_sensors.msg import PlayerLocation
import numpy as np
import sys


# INFORMATION
# This program only exists to check if player_finder is publishing its messages correctly


def callbackMsgReceived(msg):
    for i in msg.idx:
        x = np.around(msg.locations[i].pose.position.x, 3)
        y = np.around(msg.locations[i].pose.position.y, 3)

        xpix = msg.Xpixel[i]
        ypix = msg.Ypixel[i]

        frame = msg.locations[i].header.frame_id
        rospy.loginfo("Player found number " + str(i + 1) + " is in team " + msg.teams[i] + " and in position (" + str(x) + "," + str(y) + ")"
                      + " in the frame " + frame + " and pixels at (" + str(xpix) + "," + str(ypix) + ").")

def main():
    # ---------------------------------------------------
    # INITIALIZATION
    # ---------------------------------------------------
    parser = argparse.ArgumentParser(description='PSR argparse example.')
    parser.add_argument('-tn', '--topic_name', type=str, default="red1")
    args = vars(parser.parse_args())

    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/" + args["topic_name"] + "/player_location", PlayerLocation, callbackMsgReceived)

    rospy.spin()


if __name__ == '__main__':
    main()

