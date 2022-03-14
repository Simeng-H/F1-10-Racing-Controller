#!/usr/bin/env python
from collections import deque
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive


class Controller:
	def __init__(self):
		# error memory
		self.error_memory_size = 10
		self.error_memory = deque()

		# PID Control Params
		self.kp = 0.0 #TODO
		self.kd = 0.0 #TODO
		self.ki = 0.0 #TODO
		self.servo_offset = 0.0	# zero correction offset in case servo is misaligned and has a bias in turning.

		# This code can input desired velocity from the user.
		# velocity must be between [0,100] to move forward. 
		# The following velocity values correspond to different speed profiles.
		# 15: Very Slow (Good for debug mode)
		# 25: Slow and steady
		# 35: Nice Autonomous Pace
		# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
		self.vel_input = 0.0	#TODO

		rospy.init_node('pid_controller', anonymous=True)

		rospy.Subscriber("error", pid_input, self.register_pid_input)

		# Publisher for moving the car. 
		# TODO: Use the coorect topic /car_x/offboard/command.
		self.command_pub = rospy.Publisher('/car_1/offboard/command', AckermannDrive, queue_size = 1)

	def generate_control_message(self):

		angle = 0.0

		print("PID Control Node is Listening to error")
		
		## Your PID code goes here
		#TODO: Use kp, ki & kd to implement a PID controller
		
		# 1. Scale the error
		# 2. Apply the PID equation on error to compute steering
		
		# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
		command = AckermannDrive()

		# TODO: Make sure the steering value is within bounds [-100,100]
		command.steering_angle = angle

		# TODO: Make sure the velocity is within bounds [0,100]
		command.speed = vel_input

		# Move the car autonomously
		# command_pub.publish(command)
		return command

	def register_pid_input(self, pid_input):
		velocity, error = pid_input
		if len(self.error_memory) < self.error_memory_size:
			self.error_memory.append(error)
		else:
			self.error_memory.append(error)
			self.error_memory.popleft()

	def set_gains(self,kp, kd, ki, vel_input):
		self.kp = kp
		self.kd = kd
		self.ki = ki
		self.vel_input = vel_input
		
	def get_pid(self):
		pass

if __name__ == '__main__':
	kp = int(input("Enter Kp Value: "))
	kd = int(input("Enter Kd Value: "))
	ki = int(input("Enter Ki Value: "))
	vel_input = int(input("Enter desired velocity: "))

	controller = Controller()
	controller.set_gains(kp, kd, ki, vel_input)
	rospy.init_node('pid_controller', anonymous=True)
	rospy.spin()
