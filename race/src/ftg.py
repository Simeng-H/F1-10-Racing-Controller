#!/usr/bin/env python
from collections import deque
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32MultiArray

class FTGController:

    def __init__(self):
        pass
    pass

if __name__ == '__main__':
    try:
        controller = FTGController(active=False, tune=True)
    except:
        rospy.signal_shutdown()