#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rospy
from base_package.msg import jack_reset

class LimitSwitches:
    def __init__(self, init_node=False):

        if init_node:
            rospy.init_node("limit_switches", anonymous=True)

        self.channels = [21, 23, 24] # TODO: determine

        self.channel_values = [0,0,0]
        self.reset = jack_reset()

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.channels[0], GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.channels[1], GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.channels[2], GPIO.OUT, pull_up_down=GPIO.PUD_DOWN)

        GPIO.add_event_detect(self.channels[0], GPIO.RISING, callback=self.rising_callback)
        GPIO.add_event_detect(self.channels[1], GPIO.RISING, callback=self.rising_callback)
        GPIO.add_event_detect(self.channels[2], GPIO.RISING, callback=self.rising_callback)

        GPIO.add_event_detect(self.channels[0], GPIO.FALLING, callback=self.falling_callback)
        GPIO.add_event_detect(self.channels[1], GPIO.FALLING, callback=self.falling_callback)
        GPIO.add_event_detect(self.channels[2], GPIO.FALLING, callback=self.falling_callback)

        self.switches_publisher = rospy.Publisher("/base/limit_switches", jack_reset, queue_size=10)


    def rising_callback(self, channel):
        self.channel_values[self.channels.index(channel)] = 1
        self.set_reset_msg()
        self.switches_publisher.publish(self.reset)
        
    def falling_callback(self, channel):
        self.channel_values[self.channels.index(channel)] = 0
        self.set_reset_msg()
        self.switches_publisher.publish(self.reset)

    def set_reset_msg(self):
        self.reset.reset_left = self.channel_values[0]
        self.reset.reset_back = self.channel_values[1]
        self.reset.reset_right = self.channel_values[2]
