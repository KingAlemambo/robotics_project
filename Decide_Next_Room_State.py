#!/usr/bin/env python3
import rospy
import actionlib
import smach
import smach_ros
from geometry_msgs.msg import Twist
from second_coursework.srv import MoveToRoom, MoveToRoomResponse, MoveToRoomRequest
from second_coursework.msg import robotMovingAction, robotMovingFeedback
from second_coursework.srv import YOLODetectionFrame
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

class DecideNextRoom(smach.State):
    print("inside state: DecideNextRoom")

    def __init__(self):
        smach.State.__init__(self, outcomes = ['room_a','room_b','room_d', 'finished'],
                             input_keys = ['room_name','visit_counter', 'num_checks'],
                             output_keys =['room_name', 'visit_counter', 'num_checks'])

    def execute(self, userdata):


            if userdata.room_name == 'D':
                userdata.room_name = 'A'
                userdata.num_checks -= 1
                if userdata.num_checks <= 0:
                    return 'finished'
                return 'room_a'
            elif userdata.room_name == 'A':
                userdata.room_name = 'B'
                return 'room_b'
            elif userdata.room_name == 'B':
                userdata.room_name = 'D'
                return 'room_d'
            else:
                return 'aborted'

