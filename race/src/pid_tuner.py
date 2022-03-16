#!/usr/bin/env python

"""
This script requires the pynput package which can be installed through pip
"""

import rospy
from std_msgs.msg import Float32MultiArray
from pynput import keyboard
class Tuner:
    def __init__(self):
        """
        usage: u/j = +/- kp, i/k = +/- ki, o/l = +/- kd
        """

        rospy.init_node('pid_tuner', anonymous=False)
        self.publisher = rospy.Publisher('/pid_params', Float32MultiArray, queue_size=10)
        self.kp, self.ki, self.kd = 0,0,0
        print("usage: u/j = +/- kp, i/k = +/- ki, o/l = +/- kd")

        listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        listener.start()

        
        while not rospy.is_shutdown():
            # print("looping")
            pass
            
    
    def on_press(self, key):
        print("pressed")
        char = str(key.char)[0]
        # print(char, " presses")
        if char == "u":
            self.kp += 0.1
        elif char == "j":
            self.kp -= 0.1
        elif char == "i":
            self.ki += 0.1
        elif char == "k":
            self.ki -= 0.1
        elif char == "o":
            self.kd += 0.1
        elif char == "l":
            self.kd -= 0.1
        print("\nkp: %f, ki: %f, kd: %f" % (self.kp, self.ki, self.kd))
        self.publish()   
    
    def on_release(self, key):
        pass 

    def publish(self):
        print("publishing")
        message = Float32MultiArray()
        message.data = [self.kp, self.ki, self.kd]
        self.publisher.publish(message)
        print("published")


if __name__ == '__main__':
    Tuner()