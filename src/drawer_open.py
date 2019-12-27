#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped, WrenchStamped, Vector3
from control_msgs.msg import PointHeadAction, PointHeadGoal
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from ar_track_alvar_msgs.msg import AlvarMarkers
from moveit_commander import RobotCommander
import rospy
import math
import tf
import tf2_ros
import tf2_geometry_msgs
import actionlib



class Drawer:
    def __init__(self):
        rospy.init_node('drawer_open')

        self.robot = RobotCommander()
        self.arm = self.robot.get_group("arm_torso")
        self.arm.set_planning_time(0.0)
        self.arm.set_planner_id("RRTConnectkConfigDefault")
        self.arm.set_pose_reference_frame("base_footprint")
        self.arm.allow_replanning(False)

        rospy.loginfo("Manipulation started")

        rospy.loginfo("Yeah, Manipulation up and running")


        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.marker_cb)
        rospy.Subscriber("/wrist_force", Vector3, self.force_cb)

        self.grip_goal = PlayMotionGoal()
        self.markers = None
        self.force = None
        self.rad = 0.44
        self.centre_x = None
        self.centre_y = None
        self.p_final = PoseStamped()
        self.thr_x = 12.0
        self.x = []
        self.y = []
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


    def marker_cb(self, data):
        self.markers = data.markers

    def force_cb(self, value):
        self.force = value

    def drawer_door(self):

        # Now get marker position. Marker position is the position of the handle.
        rospy.loginfo("waiting for markers")
        rospy.wait_for_message("/ar_pose_marker", AlvarMarkers)
        marker = self.markers

        listener = tf.TransformListener()

        marker[0].pose.header.frame_id = "base_footprint"

        print "-"*20+"\n marker detected"
        print marker[0]


        if marker[0].pose.pose.orientation.x > 0:
            # Set marker position as goal position.


            self.p_final.header.frame_id = "base_footprint"
            self.p_final.header.stamp = rospy.Time(0)


            self.p_final.pose.position.x = marker[0].pose.pose.position.x - 0.35
            self.p_final.pose.position.y = marker[0].pose.pose.position.y - 0.11
            self.p_final.pose.position.z = marker[0].pose.pose.position.z + 0.11

            self.p_final.pose.orientation.x = 0
            self.p_final.pose.orientation.y = 0
            self.p_final.pose.orientation.z = 0
            self.p_final.pose.orientation.w = 1

            self.move_to(self.p_final)

            self.p_final.header.frame_id = "base_footprint"
            self.p_final.header.stamp = rospy.Time(0)

            self.p_final.pose.position.x = self.p_final.pose.position.x + 0.09



            self.move_to_linear(self.p_final)
            rospy.sleep(1)

            self.grip_axcli = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
            self.grip_axcli.wait_for_server()
            self.grip_goal.motion_name = 'close_gripper'
            self.grip_axcli.send_goal(self.grip_goal)
            self.grip_axcli.wait_for_result()
            rospy.sleep(2)

            self.p_final.pose.position.x = self.p_final.pose.position.x - 0.01
            self.move_to_linear(self.p_final)
            rospy.sleep(1)
            print ("Force =" + str(self.force.x))

            if self.force.x > 12:
                print "Locked!!"

            else:
                self.res = True
                print "Draw Detected"
                while self.res:
                    #Code for Drawer opening.
                    self.p_final.pose.position.x =  self.p_final.pose.position.x - 0.015
                    self.res = self.move_to_linear(self.p_final)
                    print ("Force =" + str(self.force.x))
                print "Draw Opened"

                self.grip_axcli = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
                self.grip_axcli.wait_for_server()
                self.grip_goal.motion_name = 'open_gripper'
                self.grip_axcli.send_goal(self.grip_goal)
                self.grip_axcli.wait_for_result()
                rospy.sleep(2)



        elif marker[0].pose.pose.orientation.x < 0:

            print "Door Detected"

            self.p_final.pose.position.x = marker[0].pose.pose.position.x - 0.35
            self.p_final.pose.position.y = marker[0].pose.pose.position.y + 0.03
            self.p_final.pose.position.z = marker[0].pose.pose.position.z + 0.1

            self.p_final.pose.orientation.x = 0.707
            self.p_final.pose.orientation.y = 0
            self.p_final.pose.orientation.z = 0
            self.p_final.pose.orientation.w = 0.707

            self.move_to(self.p_final)
            print self.p_final

            self.p_final.header.frame_id = "base_footprint"
            self.p_final.header.stamp = rospy.Time(0)

            self.p_final.pose.position.x = self.p_final.pose.position.x + 0.12


            self.move_to_linear(self.p_final)
            print self.p_final
            self.centre_x = self.p_final.pose.position.x
            self.centre_y = self.p_final.pose.position.y + self.rad
            rospy.sleep(1)

            self.grip_axcli = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
            self.grip_axcli.wait_for_server()
            self.grip_goal.motion_name = 'close_gripper'
            self.grip_axcli.send_goal(self.grip_goal)
            self.grip_axcli.wait_for_result()
            rospy.sleep(2)

            self.p_final.pose.position.x = self.p_final.pose.position.x - 0.02
            rospy.sleep(1)
            self.move_to_linear(self.p_final)

            if self.force.x > 12:
                print "Locked!!"

            else:
                (self.x, self.y) = self.get_circle_points(self.rad, self.centre_x, self.centre_y)
                for q in range(len(self.x)):
                    self.p_final.pose.position.x = self.x[q]
                    self.p_final.pose.position.y = self.y[q]
                    self.move_to_linear(self.p_final)
                print "Door Opened"

                self.grip_axcli = actionlib.SimpleActionClient('/play_motion', PlayMotionAction)
                self.grip_axcli.wait_for_server()
                self.grip_goal.motion_name = 'open_gripper'
                self.grip_axcli.send_goal(self.grip_goal)
                self.grip_axcli.wait_for_result()
                rospy.sleep(2)


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



    def move_to(self, pose_stamped):

        self.arm.set_start_state_to_current_state()

        self.arm.set_pose_target(pose_stamped)

        result = self.arm.go(wait=True)


        return result

    def move_to_linear(self, target_pose_stamped, jump_threshold = 0.0, velocity_scaling_factor = 1.0, min_distance = 0.0):

        eef_step = 0.01

        rospy.loginfo("Jump threshold: " + str(jump_threshold))
        rospy.loginfo("Endeffector step size in m: " + str(eef_step))

        if target_pose_stamped is None:
            rospy.logerr("pose stamped given is None.")
            return MoveItErrorCodes.FAILURE

        self.arm.set_start_state_to_current_state()

        waypoints = []

        print 6*"pose"
        print target_pose_stamped

        target_pose_stamped_base_link = self._transform_pose_to_frame(target_pose_stamped, "base_footprint")

        print 6*"moveto"
        print target_pose_stamped_base_link


        rospy.logwarn("Move linear to position {}".format(target_pose_stamped_base_link))

        waypoints.append(target_pose_stamped_base_link.pose)

        (plan, fraction) = self.arm.compute_cartesian_path(waypoints, eef_step, jump_threshold)

        plan = self.arm.retime_trajectory(self.robot.get_current_state(), plan, velocity_scaling_factor)

        rospy.loginfo("Cartesian path fraction: " + str(fraction))
        rospy.loginfo("Cartesian path minimum distance: " + str(min_distance))

        if fraction < min_distance:
            rospy.logerr("Cartesian path fraction was below 1.0 it was: " + str(fraction))
            return False
        result = self.arm.execute(plan,wait=True)
        rospy.loginfo("Move result: " + str(result))
        return result

    def _transform_pose_to_frame(self, pose_stamped, target_frame):

        # tf2 can't handle frame names with a leading / so remove it
        pose_stamped.header.frame_id = pose_stamped.header.frame_id.strip('/')
        target_frame.strip('/')

        rospy.logdebug("Transform pose \n{}\n from frame {} to frame {}".format(pose_stamped, pose_stamped.header.frame_id, target_frame))

        transform = self.tf_buffer.lookup_transform(target_frame, pose_stamped.header.frame_id, rospy.Time(0), rospy.Duration(1.0))

        pose_stamped = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

        rospy.logdebug("Transformed pose: \n{}".format(pose_stamped))

        return pose_stamped




if __name__ == '__main__':
    draw = Drawer()
    draw.drawer_door()
