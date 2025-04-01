#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

def joint_callback(msg):
    for i, name in enumerate(msg.name):
        rospy.loginfo("Encoder {}: Vị trí = {:.2f}, Vận tốc = {:.2f}".format(name, msg.position[i], msg.velocity[i]))

rospy.init_node('encoder_joint_listener')
rospy.Subscriber('/joint_states', JointState, joint_callback)

rospy.spin()
