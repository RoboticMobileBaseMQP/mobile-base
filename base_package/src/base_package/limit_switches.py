#!/usr/bin/env python3

import RPi.GPIO as GPIO
import rospy
from base_package.msg import jack_reset

class LimitSwitches:
    def __init__(self, init_node=False):

        if init_node:
            rospy.init_node("limit_switches", anonymous=True)

        self.channels = [23, 22, 21]

        self.channel_values = [0,0,0]
        self.reset = jack_reset()

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.channels[0], GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.channels[1], GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.channels[2], GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(self.channels[0], GPIO.BOTH, callback=self.event_callback, bouncetime=100)
        GPIO.add_event_detect(self.channels[1], GPIO.BOTH, callback=self.event_callback, bouncetime=100)
        GPIO.add_event_detect(self.channels[2], GPIO.BOTH, callback=self.event_callback, bouncetime=100)

        self.switches_publisher = rospy.Publisher("/base/limit_switches", jack_reset, queue_size=10)

        print("limit switches ready")
        rospy.spin()

    def event_callback(self, channel):
        # print("channel " + str(channel) + " hit")
        self.channel_values[self.channels.index(channel)] = GPIO.input(channel)
        self.set_reset_msg()
        self.switches_publisher.publish(self.reset)

    def set_reset_msg(self):
        self.reset.reset_left = self.channel_values[0]
        self.reset.reset_back = self.channel_values[1]
        self.reset.reset_right = self.channel_values[2]

if __name__ == "__main__":
    l = LimitSwitches(init_node=True)
