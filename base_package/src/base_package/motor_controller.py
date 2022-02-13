import RPi.GPIO as GPIO
import adafruit_pca9685 as ada
import board
import time
import rospy


class MotorController:
    def __init__(self, init_node=False):

        if init_node:
            rospy.init_node("motor_controller", anonymous=True)

        # setup 
        i2c_pwm = board.I2C()
        self.pwm = ada.PCA9685(i2c_pwm)

        period = .012
        self.pwm.frequency = 1/period # period = 10ms ish
        
        # setup topic listeners


    # subscriber callback
    def callback(self):
        pass

    def translate(self, value, leftMin, leftMax, rightMin, rightMax):
        # Figure out how 'wide' each range is
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - leftMin) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return rightMin + (valueScaled * rightSpan)

    # -100 to 100 input
    def set_motor_speed(self, channel, percent_speed):
        # MAX_DUTY = 0xFFFF
        # high_time = translate(percent_speed, -100, 100, .001, .002)
        # cycle = high_time*pwm.frequency*MAX_DUTY

        # backwards: 5370
        # stop: 8165
        # forwards: 10960
        cycle = self.translate(percent_speed, -100, 100, 5370, 10960) # found by guess and check

        self.pwm.channels[channel].duty_cycle = int(cycle) # 0 to 65535

# motor testing
if __name__ == "__main__":
    m = MotorController(init_node=True)

    for i in range(7):
        m.set_motor_speed(i, -100)
    time.sleep(2)
    for i in range(7):
        m.set_motor_speed(i, 0)
    time.sleep(2)
    m.set_motor_speed(100, 0)
    time.sleep(2)
    m.set_motor_speed(0, 0)