#!/usr/bin/env python3
import rospy
from second_coursework.srv import MoveToRoom, MoveToRoomRequest

print("AAAAAAAAAAAAAAAAAAAAAAAA")

rospy.init_node('move_robot_service_client')

rospy.wait_for_service('move_robot_service')

move_robot = rospy.ServiceProxy(move_robot_service, MoveToRoom)

move_robot("A")

