#!/usr/bin/env python
#F1/10 Team 1
from collections import deque
import math
import rospy
from race.msg import pid_input
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Header
import enum
import pdb
import numpy


class State(enum.Enum):
    stop = 0
    delay_turn = 1
    normal = 2

class FTGController:

    car_radius = 0.19
    max_steering_angle = math.pi/4 # 45 degrees
    safe_frontal_clearance = 0
    safe_side_clearance = 0
    # safe_side_clearance = car_radius
    full_speed_distance = 3
    min_speed = 20
    max_speed = 35
    speed_bonus = max_speed - min_speed

    def __init__(self):
        rospy.init_node('ftg_controller', anonymous=False)
        # rospy.on_shutdown(self.shutdown)
        self.command_pub = rospy.Publisher('/car_1/offboard/command', AckermannDrive, queue_size = 10)
        self.threshold_pub = rospy.Publisher('threshold', LaserScan, queue_size=10)
        self.extend_pub = rospy.Publisher('extended', LaserScan, queue_size=10)
        self.processed_pub = rospy.Publisher('processed', LaserScan, queue_size=10)
        self.state = State.stop
        self.stability_score = 0
        rospy.Subscriber("/car_1/scan", LaserScan, self.scan_listener_hook)
    
    def scan_listener_hook(self, laser_scan):
        preprocessed_ranges = self.preprocess_scan(laser_scan)
        # print("Here\n\n\n")
        # print(self.ranges)
        # pdb.set_trace()
        frontal_clearance = self.get_frontal_clearance(preprocessed_ranges)
        # side_clearance = self.get_side_clearance(preprocessed_ranges)
        self.record_stability(frontal_clearance)
        # self.determine_state(frontal_clearance, side_clearance)
        # best_ray = self.farthest_ray(preprocessed_ranges)
        best_ray = self.widest_approx_gap_midpoint_ray(preprocessed_ranges)
        # if frontal_clearance > 2:
        #     best_ray = self.farthest_ray(preprocessed_ranges)
        # else:
        #     best_ray = self.widest_approx_gap_midpoint_ray(preprocessed_ranges)

        print("best angle: %f" % (best_ray*180/math.pi))
        angle = self.generate_steering_angle(best_ray)
        speed = self.generate_speed(frontal_clearance = frontal_clearance, stability_score = self.stability_score)
        # print("speed: %f, angle %f, state %s" %(speed, angle, self.state.name))
        self.generate_and_publish_control_message(angle, speed)

    def record_stability(self, frontal_clearance):
        if(frontal_clearance < 2):
            self.stability_score = 0
        # elif(frontal_clearance < 1):
        #     self.stability_score /= 2
        else:
            self.stability_score += 1

    def preprocess_scan(self, laser_scan):
        '''
        self.ranges at the end of preprocessing should be a valid array of ranges, points where nan is reported is set to range_max
        '''
        self.raw_scan = laser_scan
        self.angle_increment = laser_scan.angle_increment
        #print("min range: %f"%laser_scan.range_min)
        # ranges = [raw if raw != float('nan') else laser_scan.range_max for raw in laser_scan.ranges]
        ranges = []
        for i in range(len(laser_scan.ranges)):
            range_ = laser_scan.ranges[i]
            if(math.isnan(range_)):
                range_ = laser_scan.range_max
                # range_ = 10
            # if range_ < 0.01:
                # print("\t zero in raw")
            # if(range_ < 0.02):
            # if(range_ < 0.02):
            #     range_ = 0.05
            elif(range_< 0.03):
                range_ = 10
            ranges.append(range_)
        # print("raw:")
        # print(laser_scan.ranges)
        # print("processed:")
        # print(laser_scan.range_max)
        #print("min range: %f"%min(ranges))
        self.ranges = ranges

        msg = LaserScan()
        msg.angle_min = self.raw_scan.angle_min
        msg.angle_max = self.raw_scan.angle_max
        msg.angle_increment = self.raw_scan.angle_increment
        msg.range_min = min(ranges)
        msg.range_max = max(ranges)
        msg.ranges = ranges
        msg.header.frame_id = "car_1_laser"

        self.processed_pub.publish(msg)

        return ranges

    def angle_to_index(self, angle):
        index = int(float(angle-self.raw_scan.angle_min)/self.raw_scan.angle_increment)
        return index

    def index_to_angle(self, index):
        # print((self.raw_scan.angle_max - self.raw_scan.angle_min) * 180 / math.pi)
        angle = index*1.0/len(self.ranges) * (self.raw_scan.angle_max - self.raw_scan.angle_min) + self.raw_scan.angle_min
        return angle
    
    def determine_state(self, frontal_clearance, side_clearance):
        # print("frontal clearance: %f, side clearance: %f" % (self.get_frontal_clearance(), self.get_side_clearance()))
        if(frontal_clearance < FTGController.safe_frontal_clearance):
            self.state = State.stop
        elif side_clearance < FTGController.safe_side_clearance:
            self.state = State.delay_turn
        else:
            self.state = State.normal
        

    def get_frontal_clearance(self, preprocessed_ranges):
        start = self.angle_to_index(-15 * math.pi/180)
        end = self.angle_to_index(15 * math.pi/180)
        # print("in get frontal clearance \n\n\n")
        # print("frontal scans \n", preprocessed_ranges)
        # print("frontal scans \n", self.raw_scan.ranges[start:end])
        return numpy.mean(preprocessed_ranges[start:end])

    def get_side_clearance(self, preprocessed_ranges):
        right_start = self.angle_to_index(-math.pi/2-math.pi/10)
        right_end = self.angle_to_index(-math.pi/2 + math.pi/10)
        left_start = self.angle_to_index(math.pi/2-math.pi/10)
        left_end = self.angle_to_index(math.pi/2 + math.pi/10)
        return min(preprocessed_ranges[right_start:right_end] + preprocessed_ranges[left_start:left_end])

    def generate_and_publish_control_message(self, angle, speed):
        command = self.make_control_message(angle, speed)
        # command = self.make_control_message(angle, speed)
        self.command_pub.publish(command)

    def generate_steering_angle(self, target_angle):
        angle = target_angle/FTGController.max_steering_angle * 100
        if angle >= 100:
            angle = 100
        if angle <= -100:
            angle = -100
        return angle
        
    def generate_speed(self, frontal_clearance, stability_score):
        # if self.state == State.stop:
        #     speed = 0
        # else:
        #     bonus_multiplier = frontal_clearance/FTGController.full_speed_distance
        #     speed = FTGController.min_speed + bonus_multiplier * FTGController.speed_bonus
        if(frontal_clearance < 1):
            print("STOPPING")
            return 0
        bonus_multiplier = math.tanh((stability_score/10.0) ** 0.5)
        speed = FTGController.min_speed + bonus_multiplier * FTGController.speed_bonus
        print("clearance: %f, stability_score: %d, speed: %f" %(frontal_clearance, stability_score, speed))
        return speed

    def make_control_message(self, angle, speed):
        command = AckermannDrive()
        command.steering_angle = angle
        command.speed = speed
        return command

    def disparity_extend(self, ranges):
        max_extend_angle = 45*math.pi/180
        num = len(ranges)
        start_range = 50
        new_ranges = [0]*start_range+ranges[start_range:]
        end_range = num
        disparity_distance = .3
        prev = ranges[start_range]
        k = start_range+1
        while k < end_range:
            if(ranges[k] > 5.5):
                # print("large value")
                pass
            try:
                curr = ranges[k]
                # left_avg = sum(ranges[k:k+5])/5
                # right_avg = sum(ranges[k-5:k])/5
                if abs(curr - prev) > disparity_distance:
                # if abs(left_avg-right_avg) > disparity_distance:
                    min_dist = min(curr,prev)
                    right_angle = min(FTGController.car_radius/min_dist,max_extend_angle)
                    left_angle = min(FTGController.car_radius/min_dist,max_extend_angle)
                    # print("right: %f"% prev, "left: %f"% curr, "disparity angle: %f"%(self.index_to_angle(k)*180/math.pi),"right angle: %f"%(right_angle*180/math.pi),"left angle: %f"%(left_angle*180/math.pi))
                    right_num_rays = int(right_angle/self.raw_scan.angle_increment)
                    left_num_rays = int(left_angle/self.raw_scan.angle_increment)
                    for i in range(k-right_num_rays,k):
                        if curr>prev:
                            break;
                        new_ranges[i] = min_dist
                        # new_ranges[i] = 0

                    for i in range(k,k+left_num_rays):
                        if curr<prev:
                            break;
                        new_ranges[i] = min_dist
                        # new_ranges[i] = 0
                    k += left_num_rays
                prev = curr
                k += 1
            except IndexError:
                k += 1

        msg = LaserScan()
        msg.angle_min = self.raw_scan.angle_min
        msg.angle_max = self.raw_scan.angle_max
        msg.angle_increment = self.raw_scan.angle_increment
        msg.range_min = min(new_ranges)
        msg.range_max = max(new_ranges)
        msg.ranges = new_ranges
        msg.header.frame_id = "car_1_laser"
        self.extend_pub.publish(msg)

        for i in range(len(new_ranges)):
            range_ = new_ranges[i]
            angle = self.index_to_angle(i) * 180/ math.pi
            if range_ > self.raw_scan.range_max-0.01:
                # print("large value at angle %f" % angle)
                pass
                

        return new_ranges

    def farthest_ray(self, ranges):
        # print("ranges: ", ranges)
        new_ranges = self.disparity_extend(ranges)
        # print("new ranges: ", new_ranges)
        start_range = self.angle_to_index(-70 * math.pi/180)
        end_range = self.angle_to_index(70 * math.pi/180)
        best_index = (start_range + end_range)//2
        best_distance = 0
        for k in range(start_range+1,end_range):
            if new_ranges[k]>best_distance:
                best_index = k
                best_distance = new_ranges[k]
        best_angle = self.index_to_angle(best_index)
        # print("best angle: %f" % (best_angle*180/math.pi))
        return best_angle

    def widest_gap_midpoint_ray(self, ranges):
        new_ranges = self.disparity_extend(ranges)
        widest_gap_start = 0
        widest_gap_end = 0
        widest_gap_width = 0

        this_gap_start = 0
        this_gap_end = 0
        this_gap_width = 0

        for i in range(1, len(new_ranges)):
            curr, prev = new_ranges[i], new_ranges[i-1]
            jump = curr-prev
            if not abs(jump) > 0.3:
                continue

            # if disparity detected
            this_gap_end = i
            this_gap_width = this_gap_end - this_gap_start
            if this_gap_width > widest_gap_width:
                widest_gap_start = this_gap_start
                widest_gap_end = this_gap_end
                widest_gap_width = this_gap_width
            this_gap_start = this_gap_end
            this_gap_end = i

        best_index = (widest_gap_start + widest_gap_end)//2
        best_angle = self.index_to_angle(best_index)
        print("best angle: %f" % best_angle)
        return best_angle

    def widest_approx_gap_midpoint_ray(self, ranges):
        new_ranges = self.disparity_extend(ranges)
        threshold = max(numpy.percentile(new_ranges,50),2)
        # print("median: %f" % median)
        threshold_ranges = [v if v>threshold else 0 for v in new_ranges]
        min_index = self.angle_to_index(-90 * math.pi/180)
        max_index = self.angle_to_index(90 * math.pi/180)
        best_start_index = min_index
        best_length = 0
        start_index = min_index
        length = 0
        for i in range(min_index,max_index):
            if(new_ranges[i]>threshold and i<max_index-1):
                length+=1
            else:
                if(length>best_length):
                    best_start_index = start_index
                    best_length = length
                start_index = i+1
                length = 0

        best_index = best_start_index+best_length//2
        best_angle = self.index_to_angle(best_index)
        # print("best angle: %f" % (best_angle*180/math.pi))

        msg = LaserScan()
        msg.angle_min = self.raw_scan.angle_min
        msg.angle_max = self.raw_scan.angle_max
        msg.angle_increment = self.raw_scan.angle_increment
        msg.range_min = min(threshold_ranges)
        msg.range_max = max(threshold_ranges)
        msg.ranges = threshold_ranges
        msg.header.frame_id = "car_1_laser"
        self.threshold_pub.publish(msg)

        return best_angle



if __name__ == '__main__':
    controller = FTGController()
    rospy.spin()
    # try:
    # except:
    #     rospy.signal_shutdown()
