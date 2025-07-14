#!/usr/bin/env python3
import rospy
from second_coursework.srv import MoveToRoom, MoveToRoomRequest

rospy.init_node('set_position', anonymous=True)

rospy.wait_for_service('move_robot_service')

set_goal_proxy = rospy.ServiceProxy('move_robot_service', MoveToRoom)

print("Before calling service")
set_goal_proxy("A")
print("After calling service")
