#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, WrenchStamped, Vector3Stamped,TransformStamped, Vector3
from std_msgs.msg import Time



class ft_frame():
    def __init__(self):
        self.pub = rospy.Publisher('/wrist_force', Vector3, queue_size = 0)
        rospy.init_node('frame', anonymous=True)
        rospy.Subscriber('/wrist_ft', WrenchStamped, self.force_cb)
        self.force = WrenchStamped()


    def force_cb(self,data):
        self.force = data

    def get_frame(self):
        while not rospy.is_shutdown():
            listener = tf.TransformListener()
            try:
                listener.waitForTransform('/base_footprint', '/wrist_ft_link', rospy.Time(0), rospy.Duration(5))
                (trans, rot) = listener.lookupTransform('/base_footprint', '/wrist_ft_link', rospy.Time(0))
            except Exception as e:
                rospy.logerr('#'*20+"\n\n"+str(e))
                return False

            # print (trans,rot)

            p = Vector3Stamped()
            p.header.stamp=rospy.Time()
            p.header.frame_id='wrist_ft_link'
            p.vector.x = self.force.wrench.force.x
            p.vector.y = self.force.wrench.force.y
            p.vector.z = self.force.wrench.force.z
            t = TransformStamped()
            t.transform.translation.x = trans[0]
            t.transform.translation.y = trans[1]
            t.transform.translation.z = trans[2]
            t.transform.rotation.x = rot[0]
            t.transform.rotation.y = rot[1]
            t.transform.rotation.z = rot[2]
            t.transform.rotation.w = rot[3]
            q=tf2_geometry_msgs.do_transform_vector3(p,t)
            self.pub.publish(q.vector)





if __name__ == '__main__':
    fra = ft_frame()
    fra.get_frame()
    rospy.spin()
