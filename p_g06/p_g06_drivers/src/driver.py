#!/usr/bin/env python3

import copy
import math
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from tf2_geometry_msgs import PoseStamped


class Driver:

    # ______________________________________________________________________________________
    # INITIALIZATIONS
    # ______________________________________________________________________________________

    def __init__(self):

        self.publisher_command = rospy.Publisher('/blue1/cmd_vel', Twist, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.sendCommandCallback)

        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goalReceivedCallback)

        self.goal = PoseStamped()
        self.goal_active = False

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.angle = 0
        self.speed = 0

    # ______________________________________________________________________________________
    # CALLBACK IF A GOAL IS RECEIVED
    # ______________________________________________________________________________________
    def goalReceivedCallback(self, msg):

        try:
            self.goal = self.tf_buffer.transform(msg, 'blue1/odom', rospy.Duration(1))
            self.goal_active = True
            rospy.logwarn('Setting new goal')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.goal_active = False
            rospy.logerr('Could not transform goal to odom frame. Ignoring this goal...')

    # ______________________________________________________________________________________
    # DRIVE TO GOAL FUNCTION
    # ______________________________________________________________________________________
    def driveStraight(self):

        # transform received goal
        goal_copy = copy.deepcopy(self.goal)
        goal_copy.header.stamp = rospy.Time.now()
        goal_in_base_link = self.tf_buffer.transform(goal_copy, 'blue1/base_footprint', rospy.Duration(1))

        # define speed and direction for the robot's movement
        self.angle = math.atan2(goal_in_base_link.pose.position.y, goal_in_base_link.pose.position.x)
        self.speed = 0.1

    # ______________________________________________________________________________________
    # CALLBACK EVERY X SECONDS
    # ______________________________________________________________________________________
    def sendCommandCallback(self, event):

        print('Sending Twist command')
        if not self.goal_active:
            self.speed = 0
            self.angle = 0

        else:
            self.driveStraight()  # sets speed and angle

        # ..............................................................
        # DEFINE TWIST MESSAGES
        # ..............................................................
        twist = Twist()
        twist.linear.x = self.speed  # m/s
        twist.angular.z = self.angle  # rad/s

        # ..............................................................
        # SEND TWIST MESSAGES
        # ..............................................................

        self.publisher_command.publish(twist)



def main():

    rospy.init_node('p_g06_driver', anonymous=False)  # initialize node

    driver = Driver()

    rate = rospy.Rate(10)  # 10 Hz frequency

    rospy.spin()


if __name__ == '__main__':
    main()

