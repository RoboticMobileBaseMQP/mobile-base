#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import sys
import Encoder
import rospy
from std_msgs.msg import Int64
from base_package.msg import effort_list, encoder_values
# import special message type of a list of 3 int64's


class JackEncoderReader:
    def __init__(self, init_node=False):
        
        if init_node:
            rospy.init_node("jack_encoder_readings", anonymous=True)
        
        self.jackEncoders = rospy.Publisher("/base/elevator_encoders", encoder_values, queue_size=10)
        # Jack Right A  = JRa
        JRa = 27
        JRb = 22
        JBa = 17
        JBb = 18
        JLa = 24
        JLb = 23

        self.encR = Encoder.Encoder(JRa, JRb)
        self.encL = Encoder.Encoder(JLa, JLb)
        self.encB = Encoder.Encoder(JBa, JBb)

        self.encoder_writer = rospy.Subscriber("/base/elevator_efforts", effort_list, self.check_encoder_reset)

    def check_encoder_reset(self, msg):
        if msg.Reset:
            self.encR.write(0)
            self.encL.write(0)
            self.encB.write(0)
    
    def home_config(self):
        # TODO
        # Read pins on limit switches to know if jack is home
        # if limit swithc is not enabled and home config is desired, lower jacks until limit switch is true
        return
        
    def main(self):
        # if home_config == True:
        #     home_config()
        values = encoder_values()
        print("publishing values")
        
        while not rospy.is_shutdown():
            
            rightJack = self.encR.read()
            leftJack = self.encL.read()
            backJack = self.encB.read()
            
            # rostopic publishing
            values.Values = [int(rightJack), int(leftJack), int(backJack)]
            self.jackEncoders.publish(values)            


if __name__ == "__main__":
    j = JackEncoderReader(init_node=True)
    j.main()
