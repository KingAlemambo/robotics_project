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



class NavigateInRoom(smach.State):
    print("inside state: NavigateInRoom")
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             output_keys=['room_name'])
        self.cmd_vel_pub= rospy.Publisher('cmd_vel', Twist, queue_size =1)

    def execute(self, userdata):
        start_time = rospy.Time.now()

        while rospy.Time.now() - start_time < rospy.Duration(30.0):
            twist = Twist()
            twist.linear.x = 0.5
            twist.angular.z = 1
            self.cmd_vel_pub.publish(twist)
            rospy.Rate(10).sleep()

        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        return 'succeeded'
