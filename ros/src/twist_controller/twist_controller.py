
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
import rospy
import time

class Controller(object):
    def __init__(self, **kwargs):
        self.map_carFeatures = {}
        for key in kwargs:
            self.map_carFeatures[key] = kwargs[key]

        self.last_update_t = time.time()


    def control(self, target_speed, target_vangular,
            current_velocity, d, pid_throttle, pid_steerangel, lpf_brake, yaw_steerangel
        ):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        current_speed = current_velocity.twist.linear.x / ONE_MPH
        current_angle = current_velocity.twist.angular.z

        cte_speed = current_speed - target_speed
        current_t = time.time()
        delta_t = current_t - self.last_update_t
        self.last_update_t = current_t
        car_throttle = pid_throttle.step(cte_speed, delta_t)
        car_steerangle = pid_steerangel.step(d, delta_t)

        throttle = 0
        brake = 0
        if car_throttle > 0:
            throttle = car_throttle
            if cte_speed < -5:
                throttle = 1.0
            if target_speed == 0.0:
                throttle = 0.0
        else:
            if car_throttle >-0.1:
                car_throttle = 0.0

            brake = -1*car_throttle

        max_car_mass = (self.map_carFeatures['vehicle_mass']+self.map_carFeatures['fuel_capacity']*GAS_DENSITY)
        brake_max = max_car_mass*-1*self.map_carFeatures['decel_limit']*self.map_carFeatures['wheel_radius']
        brake = 0.5 * brake_max * brake
        brake = lpf_brake.filt(brake)
        #rospy.logwarn("[%f, %f, %f, %f]" % (current_speed, target_speed, throttle, brake))
        #rospy.logwarn("BEGIN_ctrl_throttle===")
        #rospy.logwarn(current_velocity.twist.linear.x)
        #rospy.logwarn(cte_speed)
        #rospy.logwarn(car_throttle)
        #rospy.logwarn("END_ctrl_throttle=====")

        #rospy.logwarn("BEGIN_cte===")
        #rospy.logwarn(d)
        #rospy.logwarn("END_cte=====")

        return throttle, brake, car_steerangle
