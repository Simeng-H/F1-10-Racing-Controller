#!/usr/bin/env python
#F1/10 Team 1
from collections import deque
import math
import rospy
from race.msg import pid_input
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32MultiArray

class FTGController:

    car_radius = 0.15
    max_steering_angle = math.pi/4 # 45 degrees
    safe_distance = 0.4
    max_distance = 5
    max_speed = 40

    def __init__(self):
        rospy.init_node('ftg_controller', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        self.command_pub = rospy.Publisher('/car_1/offboard/command', AckermannDrive, queue_size = 10)
        rospy.Subscriber("/car_1/scan", LaserScan, self.scan_listener_hook)
    
    def scan_listener_hook(self, laser_scan):
        self.preprocess_and_save_scan(laser_scan)
        self.generate_and_publish_control_message()
        pass

    def preprocess_and_save_scan(self, laser_scan):
        '''
        self.ranges at the end of preprocessing should be a valid array of ranges, points where nan is reported is set to range_max
        '''
        self.angle_increment = laser_scan.angle_increment
        ranges = [raw if raw != math.nan else laser_scan.range_max for raw in laser_scan.ranges]
        self.ranges = ranges

    def generate_and_publish_control_message(self, target_angle, target_distance):
        angle = self.generate_steering_angle(target_angle)
        speed = self.generate_speed(target_distance)
        command = self.make_control_message(angle, speed)
        self.command_pub.publish(command)

    def generate_steering_angle(self, target_angle):
        angle = target_angle/FTGController.max_steering_angle * 100
        if angle >= 100:
            angle = 100
        return angle
        
    def generate_speed(self, target_distance):
        front_margin = target_distance - FTGController.safe_distance
        if front_margin < 0:
            speed = 0
        else:
            speed = front_margin / FTGController.max_distance
        if speed > FTGController.max_speed:
            speed = FTGController.max_speed
        return speed

    def make_control_message(self, angle, speed):
        command = AckermannDrive()
        command.steering_angle = angle
        command.speed = speed
        return command

    def disparity_extend(self, scan):
        num = len(scan.ranges)
        start_range = int(0.125*num) # ignore the first 30 degrees
        end_range = int(0.875*num) # ignore the last 30 degrees
        prev = scan.ranges[start_range]
        k = start_range+1
        while k < end_range:
            curr = scan.ranges[k]
            if curr - prev > 1:
                angle = (car_radius/prev)*360
                num_rays = angle*(num/240)
                for i in range(k, k+num_rays):
                    scan.ranges[i] = prev
                    k += 1
            elif prev - curr < 1:
                angle = (car_radius/prev)*360
                num_rays = angle*(num/240)
                for i in range(k-num_rays, k):
                    scan.ranges[i] = curr
            prev = curr
            k += 1

        best_ray = start_range
        best_distance = 0
        for k in range(start_range+1, end_range):
            if scan.ranges[k] > best_distance:
                best_distance = scan.ranges[k]
                best_ray = k
        return (best_ray/num)*240 - 120

if __name__ == '__main__':
    try:
        controller = FTGController()
    except:
        rospy.signal_shutdown()
