#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint
import sys
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.pose = None
        self.waypoints = None
        self.lights = []
        self.stop_wpIdx = -1
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1, buff_size=10000000)


        # : Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb, queue_size=1)
        #rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        rospy.spin()

    def pose_cb(self, msg):
        #rospy.logwarn("BEGIN===")
        #rospy.logwarn(msg)
        #rospy.logwarn("END=====")

        self.pose = msg
        #rospy.logwarn("=====BEG Current Car Position")
        #rospy.logwarn(self.pose)
        #rospy.logwarn("=====END Current Car Position")
        self.publish_final()


    def waypoints_cb(self, waypoints):
        #rospy.logwarn("BEGIN_wp===")
        #rospy.logwarn(waypoints)
        #rospy.logwarn("END_wp=====")
        self.waypoints_header = waypoints.header
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        """
            if -1,          means green light or no traffic light around
            if e.g. 292,    means stop at 292th waypoints
        """
        self.stop_wpIdx = msg.data
        #rospy.logwarn("TL is %d" % (self.stop_wpIdx))

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def publish_final(self):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        if self.waypoints is not None:
            minDist_idx = 0
            minDist_val = sys.float_info.max

            stop_pre = 3
            slowdown_interval = 20
            stop_end = 2
            min_target_speed = 3.5
            """
            Should stop at stop_wpIdx-3
            Stop from stop_wpIdx-23
            Speed 0 in [stop_wpIdx-3,stop_wpIdx+2]
            """
            if self.stop_wpIdx != -1:
                for i in range(slowdown_interval):
                    target_speed = 11.0*(slowdown_interval-i) / slowdown_interval
                    if target_speed < min_target_speed:
                        target_speed = min_target_speed

                    idx_to_set = self.stop_wpIdx-(stop_pre+slowdown_interval)+i
                    self.set_waypoint_velocity(self.waypoints, idx_to_set, target_speed)

                for i in range(stop_pre+stop_end):
                    self.set_waypoint_velocity(self.waypoints, self.stop_wpIdx-stop_pre+i, 0.0)

            for idx,wp in enumerate(self.waypoints):

                val = dl(self.pose.pose.position, wp.pose.pose.position)
                if val < minDist_val:
                    minDist_val = val
                    minDist_idx = idx


            if minDist_idx+LOOKAHEAD_WPS >= len(self.waypoints):
                l2 = self.waypoints+self.waypoints
                outList = l2[minDist_idx:minDist_idx+LOOKAHEAD_WPS]
            else:
                outList = self.waypoints[minDist_idx:minDist_idx+LOOKAHEAD_WPS]

            prev_idx = minDist_idx - 1
            if prev_idx < 0:
                prev_idx = len(self.waypoints)-2


            lane = Lane()
            lane.header = self.waypoints_header
            lane.waypoints = [self.waypoints[prev_idx]] + outList
            self.final_waypoints_pub.publish(lane)



if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
