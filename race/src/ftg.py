#!/usr/bin/env python
from collections import deque
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32MultiArray

class FTGController:

    def __init__(self):
        rospy.init_node('ftg_controller', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.command_pub = rospy.Publisher('/car_1/offboard/command', AckermannDrive, queue_size = 10)
        rospy.Subscriber("/car_1/scan", pid_input, self.scan_listener_hook)
    
    def scan_listener_hook(self):
        pass

    def generate_control_message(self):
        pass

    def disparity_extend(self, scan):
        pass

if __name__ == '__main__':
    try:
        controller = FTGController()
    except:
        rospy.signal_shutdown()