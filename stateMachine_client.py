#!/usr/bin/env python3

import roslib
roslib.load_manifest('second_coursework')
import rospy
import actionlib

from second_coursework.msg import robotMovingAction, robotMovingGoal


def client():
    print("enter client statemachine")
    rospy.loginfo('Entered statemachine client')
    rospy.init_node('room_check_client')
    client = actionlib.SimpleActionClient('robot_behavior',robotMovingAction)
    rospy.loginfo('After Simple Action client')
    client.wait_for_server()
    print("after client wait for server")

    goal = robotMovingGoal(rospy.get_param("/stateMachine_client/nchecks"))
    print("after goal")

    client.send_goal(goal)
    print("after send goal")

    client.wait_for_server()

    result = client.get_result()
    rospy.loginfo("Result: {}".format(result))
    return result


if __name__ == '__main__':
    try:
      client()
    except rospy.ROSInterruptException:
        rospy.loginfo("Programm Interrupted before completion")
