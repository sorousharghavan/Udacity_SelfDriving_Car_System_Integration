#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped, PoseStamped
import math
from styx_msgs.msg import Lane, Waypoint
from twist_controller import Controller
from yaw_controller import YawController
from pid import PID
import time

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

distance = lambda x1,y1,x2,y2: math.sqrt((x1-x2)**2 + (y1-y2)**2)

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        self.d = 0
        # TODO: Create `TwistController` object
        map_carFeatures = {
            "vehicle_mass" : vehicle_mass,
            "fuel_capacity" : fuel_capacity,
            "brake_deadband" : brake_deadband,
            "decel_limit" : decel_limit,
            "accel_limit" : accel_limit,
            "wheel_radius" : wheel_radius,
            "wheel_base" : wheel_base,
            "steer_ratio" : steer_ratio,
            "max_lat_accel" : max_lat_accel,
            "max_steer_angle" : max_steer_angle
        }
        self.controller = Controller(**map_carFeatures)

        # TODO: Subscribe to all the topics you need to.
        # The dbw_node subscribes to the /current_velocity topic
        # along with the /twist_cmd topic to receive
        # TARGET linear(40 / 3.6) and angular velocities (vy / r^2)
        self.target_state_ = {"v_linear" : 0, "v_angular" : 0}
        self.current_velocity_ = TwistStamped()
        self.dbw_isEnable_ = True

        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb, queue_size=1)
        rospy.Subscriber('/current_velocity', TwistStamped, self.currentV_cb, queue_size=1)
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbwEnable_cb, queue_size=1)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        rospy.Subscriber('/final_waypoints', Lane, self.waypoints_cb, queue_size=1)

        self.pid_throttle = PID(-0.5, -0.0001, -0.05, decel_limit, accel_limit)
        self.pid_steerangel = PID(1 , 0, 0.5, -1*(30/180.0)*math.pi, (30/180.0)*math.pi)
        self.yaw_steerangel = YawController(
                    wheel_base, steer_ratio, 2,
                    max_lat_accel, max_steer_angle
        )
        self.pred_pos = {
            "topic"          : [0, 0],
            "topic_prev"     : [0, 0],
            "topic_vel" : 0,
            "topic_angle" : 0,
            "pred"      : [0, 0],
            "pred_prev" : [0, 0],
            "last_update_pos_t"  : 0,
            "last_update_vel_t"  : 0
        }
        self.loop()

    def pose_cb(self, msg):
        self.pose = msg
        #rospy.logwarn("=====2Current Car Position: (%f, %f)" % (self.pose.pose.position.x, self.pose.pose.position.y))


    def waypoints_cb(self, msg):
        self.final_waypoints_header = msg.header
        self.final_waypoints = msg.waypoints
        self.pred_pos['topic'] = [self.pose.pose.position.x, self.pose.pose.position.y]

        # update velocity
        if (self.current_velocity_.twist.linear.x != self.pred_pos['topic_vel']) or \
           (self.current_velocity_.twist.angular.z != self.pred_pos['topic_angle']):
            self.pred_pos['last_update_vel_t'] = time.time()

        self.pred_pos['topic_vel'] = self.current_velocity_.twist.linear.x
        self.pred_pos['topic_angle'] = self.current_velocity_.twist.angular.z

        # update x-y coord
        # first init
        if self.pred_pos['last_update_pos_t'] == 0:
            self.pred_pos['topic_prev'] = self.pred_pos['topic']
            self.pred_pos['pred'] = self.pred_pos['topic']
            self.pred_pos['pred_prev'] = self.pred_pos['topic']
            self.pred_pos['last_update_pos_t'] = time.time()
            self.pred_pos['last_update_vel_t'] = time.time()

        # update value
        else:
            # need to predict location based on vel/angle, last updated location and time from update.
            if (self.pred_pos['topic'][0]==self.pred_pos['topic_prev'][0]) and \
               (self.pred_pos['topic'][1]==self.pred_pos['topic_prev'][1]):
                current_t = time.time()
                dt_v = (current_t - self.pred_pos['last_update_vel_t'])
                if dt_v == 0:
                    a_vel = 0
                    a_angle = 0
                else:
                    a_vel = (self.current_velocity_.twist.linear.x - self.pred_pos['topic_vel']) / float(dt_v)
                    a_angle = (self.current_velocity_.twist.angular.z - self.pred_pos['topic_angle']) / float(dt_v)

                avg_vel = (2*self.pred_pos['topic_vel']+a_vel*dt_v) / 2.0
                avg_ang = (2*self.pred_pos['topic_angle']+a_angle*dt_v) / 2.0

                self.pred_pos['pred'][0] = self.pred_pos['pred_prev'][0] + avg_vel*dt_v*math.cos(avg_ang)
                self.pred_pos['pred'][1] = self.pred_pos['pred_prev'][1] + avg_vel*dt_v*math.sin(avg_ang)

            # update predict location using topic information
            else:
                self.pred_pos['pred'] = self.pred_pos['topic']
                self.pred_pos['last_update_pos_t'] = time.time()

            self.pred_pos['topic_prev'] = self.pred_pos['topic']
            self.pred_pos['pred_prev']  = self.pred_pos['pred']


        n_x = self.final_waypoints[1].pose.pose.position.x -\
              self.final_waypoints[0].pose.pose.position.x
        n_y = self.final_waypoints[1].pose.pose.position.y -\
              self.final_waypoints[0].pose.pose.position.y
        x_x = self.pred_pos['pred'][0] - \
              self.final_waypoints[0].pose.pose.position.x
        x_y = self.pred_pos['pred'][1] - \
              self.final_waypoints[0].pose.pose.position.y

        proj_norm = (x_x*n_x+x_y*n_y) / (n_x*n_x+n_y*n_y)
        proj_x = proj_norm * n_x
        proj_y = proj_norm * n_y
        self.d = distance(x_x, x_y, proj_x, proj_y)

        center_x = 1559 - self.final_waypoints[0].pose.pose.position.x
        center_y = 2152 - self.final_waypoints[0].pose.pose.position.y
        centerToPos = distance(center_x, center_y, x_x, x_y)
        centerToRef = distance(center_x, center_y, proj_x, proj_y)

        if(centerToPos < centerToRef):
            self.d *= -1

        """
        rospy.logwarn("2BEGIN_pose===")
        rospy.logwarn(self.pose.pose.position)
        rospy.logwarn("END_pose===")

        rospy.logwarn("2BEGIN_prev_pos===")
        rospy.logwarn(self.final_waypoints[0].pose.pose.position)
        rospy.logwarn("END_prev_pos===")

        rospy.logwarn("2BEGIN_current_pos===")
        rospy.logwarn(self.final_waypoints[1].pose.pose.position)
        rospy.logwarn("END_current_pos===")

        rospy.logwarn("2BEGIN_d=====")
        rospy.logwarn(self.d)
        rospy.logwarn("END_d=====")
        """

    def twist_cb(self, msg):
        #rospy.logwarn("BEGIN_TS===")
        #rospy.logwarn(msg)
        #rospy.logwarn("END_TS=====")
        self.target_state_["v_linear"]  = msg.twist.linear.x
        self.target_state_["v_angular"] = msg.twist.angular.z

    def currentV_cb(self, msg):
        #rospy.logwarn("BEGIN_vel===")
        #rospy.logwarn(msg)
        #rospy.logwarn("END_vel=====")
        self.current_velocity_ = msg

    def dbwEnable_cb(self, msg):
        self.dbw_isEnable_ = msg


    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            throttle, brake, steering = self.controller.control(
                        self.target_state_['v_linear'],
                        self.target_state_['v_angular'],
                        self.current_velocity_,
                        self.d,
                        self.pid_throttle,
                        self.pid_steerangel,
                        self.yaw_steerangel
            )
            rospy.loginfo("d = %f, angle = %f" % (self.d, steering))

            if self.dbw_isEnable_:
                self.publish(throttle, brake, steering)

            rate.sleep()

    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
