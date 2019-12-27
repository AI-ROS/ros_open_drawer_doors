#!/usr/bin/env python
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import PoseStamped, WrenchStamped, Vector3
import math


class Gripper:
    def __init__(self):
        rospy.init_node('grip_close')
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.marker_cb)
        self.rad = 0.44
        self.centre_x = None
        self.centre_y = None
        self.markers = None
        self.x = None
        self.y = None
    def marker_cb(self, data):
        self.markers = data.markers

    def makecircle(self):
        rospy.loginfo("waiting for markers")
        rospy.wait_for_message("/ar_pose_marker", AlvarMarkers)
        marker = self.markers


        marker[0].pose.header.frame_id = "base_link"

        print "-"*20+"\n marker detected"
        print marker[0]
        self.centre_x = marker[0].pose.pose.position.x
        self.centre_y = marker[0].pose.pose.position.y + self.rad
        (self.x, self.y) = self.get_circle_points(self.rad, self.centre_x, self.centre_y)

        # print self.rad
        # print self.centre_x
        # print self.centre_y
        print self.x
        print self.y

    def get_circle_points(self, r, c_x, c_y):
        start_angle = math.acos((self.centre_x - c_x)/r)
        print "start_angle =" + str(start_angle)
        x = []
        y = []
        angle = start_angle + math.pi/36
        for p in range(0,15):

            x.append(c_x + r*math.cos(angle))
            y.append(c_y - r*math.sin(angle))

            angle = angle + math.pi/36

        return (x, y)







if __name__ == '__main__':
    grip = Gripper()
    grip.makecircle()
