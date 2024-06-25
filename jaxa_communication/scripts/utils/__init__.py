import time

try:
    import rospy

    NOT_ROS = False
except ImportError:
    NOT_ROS = True


class RateManager:
    def __init__(self, rate):
        self.rate = rate
        assert rate > 0, "Rate must be greater than 0"
        self.last_time = time.time()
        if not NOT_ROS:
            self.ros_rate = rospy.Rate(rate)
        self.target_elapsed_time = 1.0 / rate

    def sleep(self):
        if NOT_ROS:
            time.sleep(max(0, self.target_elapsed_time - (time.time() - self.last_time)))
            self.last_time = time.time()
        else:
            self.ros_rate.sleep()
