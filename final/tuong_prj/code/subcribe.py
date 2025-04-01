#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

def odom_callback(msg):
    vx = msg.twist.twist.linear.x  # Vận tốc thẳng (m/s)
    wz = msg.twist.twist.angular.z # Vận tốc góc (rad/s)
    
    rospy.loginfo("Vận tốc encoder: Linear = {:.2f} m/s, Angular = {:.2f} rad/s".format(vx, wz))

rospy.init_node('encoder_listener')
rospy.Subscriber('/odom', Odometry, odom_callback)

rospy.spin()
