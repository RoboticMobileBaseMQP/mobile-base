#!/usr/bin/env python3

from flask import current_app
import rospy
import smach
import time
import threading

current_task = None

# define Idle State
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['swap','task_progress', 'shutdown'],
                             output_keys=['task_progress_out'])

        self.mutex = threading.Lock()
        self.received_task = None

        task_queue_subscriber = rospy.Subscriber('/task-topic', None, self.callback) # TODO
        

    def callback(self, msg):
        self.mutex.acquire()
        self.received_task = msg.content # TODO: add to a queue
        self.mutex.release()
        pass

    def execute(self, userdata):
        global current_task
        rospy.loginfo('Sitting in Idle state')
        waiting_for_task = True

        # spin while waiting for next task
        while not waiting_for_task:
            self.mutex.acquire()
            if self.received_task: # if queue is not empty
                waiting_for_task = False
                current_task = self.received_task # pop from queue
            self.mutex.release()
        
        # after receiving task, all progress is reset
        userdata.task_progress_out = {'base': False,
                                      'elevator': False,
                                      'arm': False,
                                      'gripper': False} # might not need gripper

        if True: # if task_type is not shutdown
            if False: # if task_type==swap_arm
                return 'swap'
            else:
                return 'task_progress'
        else: # if task_type is shutdown
            return 'shutdown'

class Task_Progress(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['task_complete', 'move_base'],
                             input_keys=['task_progress_in'],
                             output_keys=['task_progress_out', 'next_value'])

    def execute(self, userdata):
        rospy.loginfo('Passing through Task_Progress state')

        # get state of current task
        

        if False: # if task complete
            return 'task_complete'

        return 'move_base'

class Base(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['check_elevator'])
    
    def execute(self, userdata):
        rospy.loginfo('Moving base in Base state')

        time.sleep(3)
        # move base code goes here

        return 'check_elevator'

class Elevator_Goal_Check(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['move_elevator', 'check_arm'])

    def execute(self, userdata):
        rospy.loginfo('Checking if elevator needs to be moved')

        # get elevator position
        
        if True: #if elevator needs to be moved 
            return 'move_elevator'
        else: # otherwise skip to arm check
            return 'check_arm'

class Elevator(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['check_arm'])
    
    def execute(self, userdata):
        rospy.loginfo('Moving jacks in Elevator state')

        time.sleep(3)
        # move elevator code goes here

        return 'check_arm'

class Arm_Goal_Check(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['move_arm', 'task_progress'])

    def execute(self, userdata):
        rospy.loginfo('Checking if arm needs to be moved')

        # get arm position
        
        if True: # if arm needs to be moved 
            return 'move_arm'
        else: # otherwise check if task is complete
            return 'task_progress'

class Arm(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['task_progress'])
    
    def execute(self, userdata):
        rospy.loginfo('Moving arm in Arm state')

        time.sleep(3)
        # move arm code goes here

        return 'task_progress'

def main():
    rospy.init_node('robot_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['SHUTDOWN']) # add ERROR state?
    sm.userdata.task_progress = {
        'base_moved': False,
        'elevator_moved': False,
        'arm_moved': False,
    }

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('IDLE', Idle(), 
                               transitions={'swap':'SHUTDOWN', 
                                            'task_progress':'TASK_PROGRESS',
                                            'shutdown':'SHUTDOWN'})
        smach.StateMachine.add('TASK_PROGRESS', Task_Progress(), 
                               transitions={'task_complete':'IDLE', 
                                            'move_base':'BASE'})


        smach.StateMachine.add('BASE', Base(), 
                               transitions={'check_elevator':'ELEVATOR_CHECK'})


        smach.StateMachine.add('ELEVATOR_CHECK', Elevator_Goal_Check(), 
                               transitions={'move_elevator':'ELEVATOR',
                                            'check_arm':'ARM_CHECK'})
        smach.StateMachine.add('ELEVATOR', Elevator(), 
                               transitions={'check_arm':'ARM_CHECK'})


        smach.StateMachine.add('ARM_CHECK', Arm_Goal_Check(), 
                               transitions={'move_arm':'ARM',
                                            'task_progress':'TASK_PROGRESS'})
        smach.StateMachine.add('ARM', Arm(), 
                               transitions={'task_progress':'TASK_PROGRESS'})

    # Create and start the introspection server
    # sis = smach_ros.IntrospectionServer('robot_state_machine', sm, '/SM_ROOT')
    # sis.start()

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    # sis.stop()


if __name__ == '__main__':
    main()