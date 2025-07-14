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


class YoloDetectionState(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'detected','tired'],
                             input_keys=['room_name', 'rule_violations'],
                             output_keys=['rule_violations'])
        rospy.loginfo("YOLODETEVTIONSTATEEEE")
        self.detect_frame_service = rospy.ServiceProxy('detect_frame', YOLODetectionFrame)
        rospy.loginfo("YOLODETEVTIONSTATEEEE1111111")
        self.detected_elements = set()


        self.speech_pub= rospy.Publisher('/speech', String , queue_size =10)
        self.feedback_pub = rospy.Publisher('/feedback', robotMovingFeedback, queue_size=10)
        self.current_pose = None
        self.pose_subscriber = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def execute(self, userdata):

        self.current_room = userdata.room_name

        start_time = rospy.Time.now()
        rospy.loginfo("before while loop")
        while rospy.Time.now() - start_time < rospy.Duration(30.0):
            rospy.loginfo("inside while loop of yolo state")

            try:
                rospy.wait_for_service('detect_frame')
                yolo_response = self.detect_frame_service()

                for detection in yolo_response.detections:
                    if detection.name == 'cat':
                        self.detected_elements.add(detection.name)

                    if detection.name == 'dog':
                        self.detected_elements.add(detection.name)

                    if detection.name == 'person':
                        self.detected_elements.add(detection.name)

                    if 'cat' in self.detected_elements and 'dog' in self.detected_elements:
                        rospy.loginfo("cat and dog ruled violated")
                        self.publish_feedback(2)
                        self.rule1_violation_flag = True
                        userdata.rule_violations[1] += 1
                        self.speech_pub.publish("Dogs and Cats are not allowed to be in the same room, they might fight")
                        return 'detected'


                    if 'person' in self.detected_elements and userdata.room_name == 'D':
                        rospy.loginfo("people and room D rule violated")
                        self.publish_feedback(1)
                        self.rule2_violation_flag = True
                        userdata.rule_violations[0] += 1
                        self.speech_pub.publish("Please can you leave room D,you are not allowed to be here")
                        return 'detected'


            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)
                return 'failed'
        print("after while loop in yolo state")
        return 'tired'

    def publish_feedback(self, rule_number):
        feedback = robotMovingFeedback()
        feedback.robot_position = self.current_pose.position
        feedback.rule_broken = rule_number

        self.feedback_pub.publish(feedback)



