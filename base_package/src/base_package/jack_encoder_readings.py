#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time
import sys
import Encoder
import rospy
from std_msgs import int64
# import special message type of a list of 3 int64's


class JackEncoderReading:
    
    def __init__(self, init_node=False):
        
        if init_node:
            rospy.init_node("jack_encoder_readings", anonymous=True)
        
        self.jackEncoders = rospy.Publisher("/base/Elevator/encoderValues", encoderValues, queue_size=10)
        # Jack Right A  = JRa
        JRa = 27
        JRb = 22
        JBa = 17
        JBb = 18
        JLa = 24
        JLb = 23

        encR= Encoder.Encoder(JRa, JRb)
        encL = Encoder.Encoder(JLa, JLb)
        encB = Encoder.Encoder(JBa, JBb)

    def home_config(self):
        # TODO
        # Read pins on limit switches to know if jack is home
        # if limit swithc is not enabled and home config is desired, lower jacks until limit switch is true
        return
        
    def main(self):
        if home_config == True:
            home_config()
        
        
        while True:
            rightJack = encR.read()
            leftJack = encL.read()
            backJack = encB.read()
            
            
            
            # rostopic publishing
            # values.Values = [int64(rightJack), int64(leftJack), int64(backJack)]
            # self.encoder_values.publish(values)
            


if __name__ == "__main__"
    encoderReadings = JackEncoderReadings()
    rospy.spin()
    

    
