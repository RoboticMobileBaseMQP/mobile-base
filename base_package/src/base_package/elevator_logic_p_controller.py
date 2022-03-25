#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from base_package.msg import effort_list, encoder_values, jack_reset
import threading
import time
import math


class ElevatorNode:
    def __init__(self, init_node=False):

        if init_node:
            rospy.init_node("elevator_logic", anonymous=True)

        # Listen to the xbox controller
        rospy.Subscriber("/joy", Joy, self.joy_callback)

        # Listen to reported encoder values
        rospy.Subscriber("/base/elevator_encoders", encoder_values, self.encoder_callback)

        self.set_point = 0 # point the jacks are trying to reach
        self.set_point_delta = 0 # change in set_point
        self.delta_scalar = 600 # rate of set_point change # TODO: play with this value

        # Publish calculated efforts to low level controllers
        self.elevator_efforts = rospy.Publisher("/base/elevator_efforts", effort_list, queue_size=10)
        self.efforts = effort_list()

        # setup thread for updating set_point
        self.mutex = threading.Lock()
        self.set_point_updater = threading.Thread(target=self.update_set_point)
        self.set_point_updater.start()
        self.thread_running = True

        self.jacks_zeroed = False

        # Publish updates to Simulation
        # arm_name = rospy.get_param("arm")
        # arm_namespace = "panda" if arm_name=="panda" else "my_gen3"
        # arm_controller_str = "/" + arm_namespace + "/" + arm_name + "_{0}_joint_controller/command"

        # self.zInsertController = rospy.Publisher(arm_controller_str.format('z'), Float64, queue_size=10)

    def joy_callback(self, msg):
        if msg.buttons[7]:
            print("resetting encoders")
            j = jack_reset()
            j.reset_left, j.reset_back, j.reset_right = 1, 1, 1
            self.elevator_resets.publish(j)

        self.mutex.acquire()
        self.set_point_delta = self.delta_scalar*msg.axes[7]
        self.mutex.release()

    def update_set_point(self):
        while True:
            self.mutex.acquire()
            self.set_point += self.set_point_delta
            self.mutex.release()
            time.sleep(.1)
            if not self.thread_running:
                break

    # switches between zeroing method to p controller method after jacks are zeroed 
    def encoder_callback(self, msg):
        if self.jacks_zeroed:
            self.jack_p_controller(msg)
        else:
            self.zero_jacks(msg)

    # brings all 3 jacks down until they hit limit switches
    def zero_jacks(self, msg):
        pass
        # get all jacks down to zero
        self.jacks_zeroed = True

    # P controller for each jack motor
    def jack_p_controller(self, msg):
        # TODO: Publish rough coordinates of elevator to update in sim!

        # TODO: play with these values 
        C1, C2 = .7, .3
        temp = []

        encoders = msg.Values

        debug_str = "curr pt: " + str(encoders) + ", set pt: " + str(self.set_point)

        for jack_pos in encoders:
            set_pt_offset = self.set_point - jack_pos  # how far away is the jack from its set point
            avg_jack_offset = sum(encoders)/len(encoders) - jack_pos # how far away is the jack from the avg of all 3 jacks

            effort = C1*self.sigmoid(set_pt_offset) + C2*self.sigmoid(avg_jack_offset) 
            temp.append(max(min(effort, 100), -100))
        
        debug_str += ", efforts: " + str(temp)
        self.efforts.Efforts = [-x for x in temp] # motors spin in reverse of given efforts (+ effort => move down)

        print(debug_str)
        self.elevator_efforts.publish(self.efforts)

    def sigmoid(self, x):
        return 200 / (1 + math.exp(-x/2000)) - 100


if __name__=="__main__":
    e = ElevatorNode(init_node=True)
    rospy.spin()

    e.thread_running = False

    kill_efforts = effort_list()
    kill_efforts.Efforts = [0.0, 0.0, 0.0]
    e.elevator_efforts.publish(kill_efforts) # doesn't work bc not in rospy.spin() loop?

