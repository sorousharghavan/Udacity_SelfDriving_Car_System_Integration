import rospy

MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0
        self.last_int_val = 0.0

    def step(self, error, sample_time):
        self.last_int_val = self.int_val

        integral = self.int_val + error * sample_time;
        derivative = (error - self.last_error) / sample_time;

        y = self.kp * error + self.ki * self.int_val + self.kd * derivative;

        rospy.logwarn("BEGIN_PID_INFO===")
        #rospy.logwarn(self.pose)
        rospy.logwarn("KP = %f", self.kp)
        rospy.logwarn("ERROR = %f", error)
        rospy.logwarn("KI = %f", self.ki)
        rospy.logwarn("Int = %f", self.int_val)
        rospy.logwarn("KD = %f", self.kd)
        rospy.logwarn("DERVIATE = %f", derivative)
        rospy.logwarn("ANGLE = %f", y)


        val = max(self.min, min(y, self.max))
        rospy.logwarn("Max=%f, Min=%f" % (self.max, self.min))
        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error
        rospy.logwarn("ANGLE_TURN = %f", val)
        rospy.logwarn("END_PID_INFO=====")
        return val
