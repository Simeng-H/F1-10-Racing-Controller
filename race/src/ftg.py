#!/usr/bin/env python
from collections import deque
import math
import rospy
from race.msg import pid_input
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32MultiArray

class FTGController:

    def __init__(self):
        rospy.init_node('ftg_controller', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.command_pub = rospy.Publisher('/car_1/offboard/command', AckermannDrive, queue_size = 10)
        rospy.Subscriber("/car_1/scan", LaserScan, self.scan_listener_hook)
    
    def scan_listener_hook(self, scan):
        pass

    def preprocess_and_save_scan(self, laser_scan):
        '''
        self.ranges at the end of preprocessing should be a valid array of ranges, points where nan is reported is set to range_max
        '''
        self.angle_increment = laser_scan.angle_increment
        ranges = [raw if raw != math.nan else laser_scan.range_max for raw in laser_scan.ranges]
        self.ranges = ranges

    def disparity_extend(self, scan):
        pass

if __name__ == '__main__':
    try:
        controller = FTGController()
    except:
        rospy.signal_shutdown()