#!/usr/bin/env python3

import rospy
import actionlib
import smach
import smach_ros
from Decide_Next_Room_State import  DecideNextRoom
from Go_To_RoomState import GoToRoom
from Navigate_In_Room_State import NavigateInRoom
from Yolo_Detection_State import YoloDetectionState
from second_coursework.msg import robotMovingAction, robotMovingFeedback, robotMovingResult




class RobotBehaviorServer:
    print("inside state machine")
    def __init__(self):
        self.behavior_server= actionlib.SimpleActionServer('robot_behavior', robotMovingAction, self.execute, False)
        self.feedback_subscriber = rospy.Subscriber('/feeedback', robotMovingFeedback, self.feedback_callback)
        self.behavior_server.start()
        self.rule_violations = [0, 0]
        self.robot_position = None
        self.rule_broken = None

    def feedback_callback(self, feedback_msg):
        self.robot_position = feedback_msg.robot_position
        self.rule_broken = feedback_msg.rule_broken



    def execute(self, goal):
        sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'stop'])
        sm.userdata.room_name = 'A'
        sm.userdata.room_a_count = 0
        sm.userdata.room_b_count = 0
        sm.userdata.visit_counter = 0
        sm.userdata.rule_violations =[0,0]
        sm.userdata.num_checks = goal.num_checks

        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()




        with sm:
            smach.StateMachine.add('GO_TO_ROOM', GoToRoom(),
                                   transitions={'succeeded': 'CONCURRENT_STATES', 'failed': 'aborted'},
                                   remapping={'room_name': 'room_name'})  #

            concurrence = smach.Concurrence(outcomes=['succeeded', 'failed', 'detected'],
                                            default_outcome='failed',
                                            input_keys= ['room_name','rule_violations'],
                                            output_keys=['room_name', 'rule_violations'],
                                            outcome_map={'succeeded': {'NAVIGATE_IN_ROOM': 'succeeded'},
                                                         'detected': {'YOLO_DETECTION': 'detected'}})

            with concurrence:
                smach.Concurrence.add('NAVIGATE_IN_ROOM', NavigateInRoom())
                smach.Concurrence.add('YOLO_DETECTION', YoloDetectionState())

            sis1 = smach_ros.IntrospectionServer('server_name', concurrence, '/SM_ROOT/CONCURRENT_STATES')
            sis1.start()

            smach.StateMachine.add('DECIDE_NEXT_ROOM', DecideNextRoom(),
                               transitions={'room_a': 'GO_TO_ROOM',
                                             'room_b': 'GO_TO_ROOM',
                                            'room_d': 'GO_TO_ROOM',
                                            'finished': 'stop'},
                               remapping={'room_name': 'room_name',
                                          'visit_counter': 'visit_counter'})

            smach.StateMachine.add('CONCURRENT_STATES', concurrence, transitions={
                'succeeded': 'DECIDE_NEXT_ROOM',
                'failed': 'aborted',
                'detected': 'DECIDE_NEXT_ROOM'})

        outcome = sm.execute()
        result = robotMovingResult()
        result.rule_violations = sm.userdata.rule_violations
        sis.stop()
        sis1.stop()
        rospy.loginfo("rule_violation 1 was broken: %s" %sm.userdata.rule_violations[0])
        rospy.loginfo("rule_violation 2 was broken: %s" %sm.userdata.rule_violations[1])

        self.behavior_server.set_succeeded(result)


if __name__ == '__main__':
    print("HELOOOOOOOOOOOO")
    rospy.init_node('room_behavior_server')
    behavior_server= RobotBehaviorServer()
    rospy.spin()