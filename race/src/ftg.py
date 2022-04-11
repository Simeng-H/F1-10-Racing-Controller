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
    full_speed_distance = 5
    max_speed = 40

    def __init__(self):
        rospy.init_node('ftg_controller', anonymous=False)
        # rospy.on_shutdown(self.shutdown)
        self.command_pub = rospy.Publisher('/car_1/offboard/command', AckermannDrive, queue_size = 10)
        rospy.Subscriber("/car_1/scan", LaserScan, self.scan_listener_hook)
    
    def scan_listener_hook(self, laser_scan):
        self.preprocess_and_save_scan(laser_scan)
        self.disparity_extend(self.ranges)
        # self.generate_and_publish_control_message()
        pass

    def preprocess_and_save_scan(self, laser_scan):
        '''
        self.ranges at the end of preprocessing should be a valid array of ranges, points where nan is reported is set to range_max
        '''
        self.angle_increment = laser_scan.angle_increment
        ranges = [raw if raw != float('nan') else laser_scan.range_max for raw in laser_scan.ranges]
        self.ranges = ranges

    def angle_to_index(self, angle):
        index = int(float(angle-self.raw_scan.angle_min)/self.raw_scan.angle_increment)
        return index

    def get_frontal_clearance(self):
        start = self.angle_to_index(-math.pi/10)
        end = self.angle_to_index(math.pi/10)
        return min(self.ranges[start:end])

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
            speed = front_margin / FTGController.full_speed_distance
        if speed > FTGController.max_speed:
            speed = FTGController.max_speed
        return speed

    def make_control_message(self, angle, speed):
        command = AckermannDrive()
        command.steering_angle = angle
        command.speed = speed
        return command

    def disparity_extend(self, ranges):
        num = len(ranges)
        start_range = int(0.125*num) # ignore the first 30 degrees
        end_range = int(0.875*num) # ignore the last 30 degrees
        prev = ranges[start_range]
        k = start_range+1
        while k < end_range:
            curr = ranges[k]
            if curr - prev > 1:
                angle = (FTGController.car_radius/prev)
                num_rays = self.raw_scan.angle_increment/angle
                for i in range(k, k+num_rays):
                    ranges[i] = prev
                    k += 1
            elif prev - curr < 1:
                angle = (FTGController.car_radius/curr)
                num_rays = self.raw_scan.angle_increment/angle
                for i in range(k-num_rays, k):
                    # print(ranges[i])
                    # print(curr)
                    ranges[i] = curr
            prev = curr
            k += 1

        best_ray = start_range
        best_distance = 0
        for k in range(start_range+1, end_range):
            if ranges[k] > best_distance:
                best_distance = ranges[k]
                best_ray = k

        print("best angle: %f" % (best_ray/num)*240 - 120)
        return (best_ray/num)*240 - 120

if __name__ == '__main__':
    controller = FTGController()
    rospy.spin()
    # try:
    # except:
    #     rospy.signal_shutdown()
