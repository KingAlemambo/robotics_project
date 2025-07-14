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

class GoToRoom(smach.State):
    print("inside state: GoToRoom")
    def __init__(self):
        smach.State.__init__(self, outcomes = ['succeeded', 'failed'],
                             input_keys =['room_name',],
                             output_keys =['room_name', ])
        self.counter = 0
        self.service_client = rospy.ServiceProxy('move_robot_service', MoveToRoom)
        rospy.wait_for_service('move_robot_service')


    def execute(self, userdata):
        rospy.loginfo('Executing state GoToRoom')
        try:
            request = MoveToRoomRequest()
            request.room_name = userdata.room_name
            response =  self.service_client(request)


            if response.success:
                return 'succeeded'
            else:
                return 'failed'
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return 'failed'