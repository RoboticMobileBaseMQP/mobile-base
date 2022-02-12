# from adafruit_servokit import ServoKit
# import 
# kit = ServoKit(channels=16)

# kit.servo[0].set_pulse_width_range(

import RPi.GPIO as GPIO
import adafruit_pca9685 as ada
import board
import time

i2c_pwm = board.I2C()
pwm = ada.PCA9685(i2c_pwm)

period = .012
pwm.frequency = 1/period # period = 10ms ish



def translate(value, leftMin, leftMax, rightMin, rightMax):
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)

# -100 to 100 input
def set_motor_speed(percent_speed):
#     MAX_DUTY = 0xFFFF
#     high_time = translate(percent_speed, -100, 100, .001, .002)
#     cycle = high_time*pwm.frequency*MAX_DUTY
    cycle = translate(percent_speed, -100, 100, 5370, 10960) # found by guess and check
    
    print(cycle)
    pwm.channels[0].duty_cycle = int(cycle) # 0 to 65535

set_motor_speed(-100)
time.sleep(2)
set_motor_speed(0)
time.sleep(2)
set_motor_speed(100)
time.sleep(2)
set_motor_speed(0)
# 
# for i in range(-100, 100, 5):
#     set_motor_speed(i)
#     time.sleep(1)

# backwards: 5370
# stop: 8165
# forwards: 10960
