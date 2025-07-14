#!/usr/bin/env python3
import math
import rospy
import actionlib
from second_coursework.srv import MoveToRoom, MoveToRoomResponse, MoveToRoomRequest
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from actionlib_msgs.msg import GoalStatus

rospy.init_node('move_robot_server')

room_coordinates ={
    'A': (1.86,8.53,0),
    'B': (5.95,8.46,0),
    'D': (1.71,3.31,0)
}
def move_robot(x,y,z):
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    move_base_client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z

    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = math.sin(-math.pi/4)
    goal.target_pose.pose.orientation.w = math.cos(-math.pi/4)



    move_base_client.send_goal(goal)
    move_base_client.wait_for_result()

    status = move_base_client.get_state()

    return status == GoalStatus.SUCCEEDED

def handle_move_to_room(req: MoveToRoomRequest):
    print("Hello")
    room_name = req.room_name
    rospy.loginfo(f"Received request to move to room: {room_name}")

    if room_name in room_coordinates:
        x,y,z = room_coordinates[room_name]
        success = move_robot(x,y,z)
        rospy.loginfo(f"Move result: {success}")
        return MoveToRoomResponse(success)

    else:
        rospy.logwarn(f"Requested room {room_name} does not exist.")
        return MoveToRoomResponse(False)

s= rospy.Service('move_robot_service', MoveToRoom, handle_move_to_room)
rospy.spin()

