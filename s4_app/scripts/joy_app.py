#! /usr/bin/env python

import rospy
import actionlib
from s4_msgs.msg import *

if __name__ == '__main__':
    rospy.init_node('joy_app')

    # activate joy_navigation 
    navigation_client = actionlib.SimpleActionClient('joy_navigation', GameAppAction)
    navigation_client.wait_for_server()
    goal = GameAppGoal()
    navigation_client.send_goal(goal)

    firecontrol_client = actionlib.SimpleActionClient('joy_firecontrol', GameAppAction)
    firecontrol_client.wait_for_server()
    goal = GameAppGoal()
    firecontrol_client.send_goal(goal)

    rospy.spin()