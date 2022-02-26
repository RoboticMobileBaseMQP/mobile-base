#!/usr/bin/env python3

import rospy
import smach
import time

# define Idle State
class Idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['swap','task_progress', 'shutdown'])

    def execute(self, userdata):
        rospy.loginfo('Sitting in Idle state')

        # poll for a task?
        # read from userdata?
        
        if True: # if there is a task
            if False: # if task=swap_arm
                return 'swap'
            else:
                return 'task_progress'
        else: # if no task
            return 'shutdown'

class Task_Progress(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['task_complete', 'move_base'])

    def execute(self, userdata):
        rospy.loginfo('Passing through Task_Progress state')

        # get state of current task
        if False: # if task complete
            return 'task_complete'

        return 'move_base'

class Base(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['move_elevator'])
    
    def execute(self, userdata):
        rospy.loginfo('Moving base in Base state')

        time.sleep(3)
        # move base code goes here

        return 'move_elevator'

class Elevator(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['move_arm'])
    
    def execute(self, userdata):
        rospy.loginfo('Moving jacks in Elevator state')

        time.sleep(3)
        # move elevator code goes here

        return 'move_arm'

class Arm(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['task_complete'])
    
    def execute(self, userdata):
        rospy.loginfo('Moving arm in Arm state')

        time.sleep(3)
        # move arm code goes here

        return 'task_complete'

def main():
    rospy.init_node('robot_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['SHUTDOWN']) # add ERROR state?

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
                               transitions={'move_elevator':'ELEVATOR'})
        smach.StateMachine.add('ELEVATOR', Elevator(), 
                               transitions={'move_arm':'ARM'})
        smach.StateMachine.add('ARM', Arm(), 
                               transitions={'task_complete':'TASK_PROGRESS'})
        


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()