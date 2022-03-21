#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

# Some useful variable declarations.
angle_range = 240	# Hokuyo 4LX has 240 degrees FoV for scan
forward_projection = 0.02*vel	# distance (in m) that we project the car forward for correcting the error. You have to adjust this.
desired_distance = 0.25	# distance from the wall (in m). (defaults to right wall)
vel = 15 		# this vel variable is not really used here.
error = 0.0		# initialize the error
car_length = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

# Handle to the publisher that will publish on the error topic, messages of the type 'pid_input'
pub = rospy.Publisher('error', pid_input, queue_size=10)


def getRange(data,angle):
	# data: single message from topic /scan
    # angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
    # Outputs length in meters to object with angle in lidar scan field of view
    # Make sure to take care of NaNs etc.

	angle -= math.pi/2
	index = int(float(angle-data.angle_min)/data.angle_increment)
	try:
		val = data.ranges[index]
	except IndexError: # BAD INDEX
		# return data.range_max#math.nan
		return float("nan")

	return val
	# if data.range_min<=val<=data.range_max:
	# 	return val
	# else: # BAD LIDAR data
	# 	return data.range_max#math.nan

def fit_line(points):
	"""
	calculates a line of best fit through points using the normal equation
	@param points: list of 2-tuples, each representing a point (x,y)
	"""
	length = len(points)
	X0 = [1.0 for point in points]
	X1 = [point[0] for point in points]
	Y = [point[1] for point in points]
	xtx = [
		[sum([x0**2 for x0 in X0]), 					sum([X0[i] * X1[i] for i in range(length)])	],
		[sum([X0[i] * X1[i] for i in range(length)]), 	sum([x1**2 for x1 in X1])					]
	]
	det = (xtx[0][0]*xtx[1][1] - xtx[0][1]*xtx[1][0])
	xtx_inv = [
		[xtx[1][1]/det,		-xtx[0][1]/det	],
		[-xtx[1][0]/det,	xtx[0][0]/det	]
	]
	xty = [
		sum([X0[i] * Y[i] for i in range(length)]),
		sum([X1[i] * Y[i] for i in range(length)])
	]
	
	t0 = sum([xtx_inv[0][0]*xty[0], xtx_inv[0][1]*xty[1]])
	t1 = sum([xtx_inv[1][0]*xty[0], xtx_inv[1][1]*xty[1]])

	return t0, t1

def get_dist(line):
	c,m = line
	angle = get_angle(line)
	dist = c * math.sin(angle)
	return dist

def get_angle(line):
	_, m = line
	return math.atan(-1.0/m)

def polar_to_cartesian(dist, ang):
	"""
	angle assumed to be in (-pi/2, pi/2)
	"""
	x = dist * math.cos(ang)
	y = dist * math.sin(ang)
	return x, y

def callback(data):
	global forward_projection

	angles_deg = [-10, 10, 30, 50]
	angles = [math.radians(angle) for angle in angles_deg]
	dists = [getRange(data, angle) for angle in angles]

	points = []
	for i in range(len(angles)):
		dist = dists[i]
		angle = angles[i]
		if math.isnan(dist):
			continue
		x,y = polar_to_cartesian(dist, angle)
		points.append([x,y])
	
	line = fit_line(points)
	current_dist = get_dist(line)
	angle = get_angle(line)
	projected_dist = current_dist - forward_projection * math.tan(angle)

	msg = pid_input()	# An empty msg is created of the type pid_input
	# this is the error that you want to send to the PID for steering correction.
	msg.pid_error = desired_distance - projected_dist	
	pub.publish(msg)


if __name__ == '__main__':
	print("Hokuyo LIDAR node started")
	rospy.init_node('dist_finder',anonymous = True)
	# TODO: Make sure you are subscribing to the correct car_x/scan topic on your racecar
	rospy.Subscriber("/car_1/scan",LaserScan,callback)
	rospy.spin()
