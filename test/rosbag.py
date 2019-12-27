#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3, WrenchStamped
import numpy as np
x = []
y = []
z = []
def force_cb(data):
     x.append(data.wrench.force.x)
     y.append(data.wrench.force.y)
     z.append(data.wrench.force.z)
     print "Force x \n"
     print x
     print "Force y \n"
     print y
     print "Force z \n"
     print z

if __name__ == '__main__':
    rospy.init_node('rosbag')
    rospy.Subscriber('/wrist_ft', WrenchStamped, force_cb)
    rospy.spin()
