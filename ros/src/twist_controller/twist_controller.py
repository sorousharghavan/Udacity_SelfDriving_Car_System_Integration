
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
import rospy

class Controller(object):
    def __init__(self, **kwargs):
        self.map_carFeatures = {}
        for key in kwargs:
            self.map_carFeatures[key] = kwargs[key]


    def control(self, target_speed, target_vangular,
            current_velocity, d, pid_throttle, pid_steerangel, yaw_steerangel
        ):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        current_speed = current_velocity.twist.linear.x / ONE_MPH
        current_angle = current_velocity.twist.angular.z

        cte_speed = current_speed - target_speed
        car_throttle = pid_throttle.step(cte_speed, 1/50.0)
        car_steerangle = pid_steerangel.step(d, 1/50.0)

        #car_steerangle += yaw_steerangel.get_steering(
    #        target_speed, target_vangular, current_speed
    #    )

        throttle = 0
        brake = 0
        if car_throttle > 0:
            throttle = car_throttle
        else:
            brake = -1*car_throttle

        #rospy.logwarn("BEGIN_ctrl_throttle===")
        #rospy.logwarn(current_velocity.twist.linear.x)
        #rospy.logwarn(cte_speed)
        #rospy.logwarn(car_throttle)
        #rospy.logwarn("END_ctrl_throttle=====")

        #rospy.logwarn("BEGIN_cte===")
        #rospy.logwarn(d)
        #rospy.logwarn("END_cte=====")

        return throttle, brake, car_steerangle
