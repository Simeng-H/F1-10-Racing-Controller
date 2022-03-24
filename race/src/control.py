#!/usr/bin/env python
from collections import deque
import math
import rospy
from race.msg import pid_input
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32MultiArray

class Controller:
	default_kp = 2.5
	default_kd = 0.1
	default_ki = 0
	default_speed = 20.0
	speed_range = 20.0
	servo_offset = 0.0
	active_frequency = 10

	high_error_threshold = 0.6
	low_error_threshold = 0.15

	def __init__(self, active=False, tune=False):
		"""
		@param active: whether to run in active mode, if in active mode, node publishes at fixed interval asynchronously with respect to receiving input, if in passive mode, node publishes each time input is received
		@param tune: listens to /pid_params if set to True. To be used with pid_tuner
		"""

		self.time = 0
		self.stable_time = 0
		# error memory
		self.error_memory_size = 20
		self.error_memory = deque()

		# PID Control Params
		self.kp = Controller.default_kp
		self.kd = Controller.default_kd
		self.ki = Controller.default_ki
		self.default_speed = Controller.default_speed
		self.speed_range = Controller.speed_range
		self.angle = 0.0

		# optionals
		self.active = active
		if self.active:
			# Active frequency
			self.frequency = Controller.active_frequency
			self.interval = 1/self.frequency

		self.tune = tune
		if self.tune:
			rospy.Subscriber("pid_params", Float32MultiArray, self.pid_params_hook)

		# This code can input desired velocity from the user.
		# velocity must be between [0,100] to move forward. 
		# The following velocity values correspond to different speed profiles.
		# 15: Very Slow (Good for debug mode)
		# 25: Slow and steady
		# 35: Nice Autonomous Pace
		# > 40: Careful, what you do here. Only use this if your autonomous steering is very reliable.
		self.vel_input = Controller.default_speed

		# node config
		rospy.init_node('pid_controller', anonymous=False)
		rospy.on_shutdown(self.shutdown)


		rospy.Subscriber("error", pid_input, self.error_listener_hook)

		# Publisher for moving the car. 
		self.command_pub = rospy.Publisher('/car_1/offboard/command', AckermannDrive, queue_size = 10)

		if active:
			self.run_active()


	def pid_params_hook(self, message):
		# print("registering PID params")
		data = message.data
		self.kp, self.ki, self.kd = data
		# print("kp: %f, ki: %f, kd: %f" % (self.kp, self.ki, self.kd))

	def generate_control_message(self):
		# An empty AckermannDrive message is created. You will populate the steering_angle and the speed fields.
		command = AckermannDrive()

		# print("PID Control Node is Listening to error")
		
		## Your PID code goes here
		
		# 1. Scale the error
		# 2. Apply the PID equation on error to compute steering

		p,i,d = self.get_pid()
		pid_output = self.kp * p + self.ki * i + self.kd * d

		# print("pid_output: ",pid_output)
		
		# convert pid_output_to steering angle through tanh function, *100 since range is [-100,100]
		self.angle = (math.tanh(pid_output))*100

		command.steering_angle = self.angle
		command.speed = self.get_speed()

		print("error: %f, angle: %f, speed: %f" % (self.error_memory[-1], command.steering_angle, command.speed))

		return command

	def register_pid_input(self, pid_input):
		# return
		self.time += 1
		error = pid_input.pid_error
		self.record_stability(error)
		self.error_memory.append(error)
		if len(self.error_memory) >= self.error_memory_size:
			self.error_memory.popleft()

	def record_stability(self, error):
		if(error > Controller.high_error_threshold):
			self.stable_time = 0
		elif(error > Controller.low_error_threshold):
			self.stable_time /= 2
		else:
			self.stable_time += 1

	def set_gains(self,kp, kd, ki, vel_input):
		self.kp = kp
		self.kd = kd
		self.ki = ki
		self.default_speed = vel_input
		
	def get_pid(self):
		if len(self.error_memory) == 0:
			return 0, 0, 0
		if len(self.error_memory) == 1:
			return self.error_memory[0], 0, 0
		p = self.error_memory[-1]
		i = sum(self.error_memory)
		d = self.error_memory[-1] - self.error_memory[-2]
		if len(self.error_memory) > 4:
			d = sum([
				self.error_memory[-1] * 11/6.0,
				self.error_memory[-2] * -3,
				self.error_memory[-3] * 3/2,
				self.error_memory[-4] * 1,
			])
		return p,i,d

	def get_speed(self):
		# return 25
		p,i,d = self.get_pid()
		pid_output = self.get_pid_output()

		# threshold
		bonus = math.tanh(self.stable_time ** 0.5)
		speed_bonus = bonus * self.speed_range


		# err = 1
		# if len(self.error_memory) > 0:
		# 	err = sum([abs(x) for x in self.error_memory])/len(self.error_memory)
		# err = abs(d)
		# speed_bonus = 1/(1*err+1) * 15
		speed = self.default_speed + speed_bonus
		# print("speed: %f error: %f time: %d" % (speed, err, self.time))
		print("speed: %f stable time: %f time: %d" % (speed, self.stable_time, self.time))
		return speed

	def get_pid_output(self):
		p,i,d = self.get_pid()
		pid_output = self.kp * p + self.ki * i + self.kd * d
		# pid_output = self.kp * p
		return pid_output

	def run_active(self):
		while not rospy.is_shutdown():
			control_msg = self.generate_control_message()
			self.command_pub.publish(control_msg)
			rospy.sleep(self.interval)

	def error_listener_hook(self, pid_input):
		self.register_pid_input(pid_input)

		# If in passive mode: need listener to trigger publication
		if not self.active:
			control_msg = self.generate_control_message()
			self.command_pub.publish(control_msg)

	def shutdown(self):
		rospy.sleep(0.1)
		print("Shutting down")


if __name__ == '__main__':
	# kp = int(input("Enter Kp Value: "))	
	# kd = int(input("Enter Kd Value: "))
	# ki = int(input("Enter Ki Value: "))
	# vel_input = int(input("Enter desired velocity: "))
	try:
		controller = Controller(active=False, tune=True)
	except:
		rospy.signal_shutdown()
	
	rospy.spin()
	# controller.set_gains(kp, kd, ki, vel_input)
