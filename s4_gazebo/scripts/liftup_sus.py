#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

if __name__ == '__main__':
    rospy.init_node('liftup_sus')
    pub0 = rospy.Publisher('omni0/wheel0_sus/command', Float64, queue_size=10)
    pub1 = rospy.Publisher('omni0/wheel1_sus/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('omni0/wheel2_sus/command', Float64, queue_size=10)

    rospy.sleep(5.0)
    r = rospy.Rate(50)
    i = 0
    while not rospy.is_shutdown():
        command_pos= (100-i) * 0.0002
        pub0.publish(command_pos)
        pub1.publish(command_pos)
        pub2.publish(command_pos)
        #print(command_pos)
        if command_pos<=0:
            break
        i+=1
        r.sleep()
    rospy.loginfo("Liftup Done")


