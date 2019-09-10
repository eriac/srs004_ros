#! /usr/bin/env python

import rospy
import actionlib
from s4_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node('joy_app')

    navigation_client = actionlib.SimpleActionClient('joy_navigation', GameAppAction)
    navigation_client.wait_for_server()

    goal = GameAppGoal()

    navigation_client.send_goal(goal)
    rospy.spin()
    #rospy.sleep(5.0)
    #navigation_client.cancel_all_goals()