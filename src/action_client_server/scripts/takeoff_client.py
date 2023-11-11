#!/usr/bin/env python

import rospy
import actionlib
from action_client_server.msg import TakeoffAction, TakeoffGoal, TakeoffResult, TakeoffFeedback
from geometry_msgs.msg import PoseStamped

def feedback_callback(feedback):
    # process feedback here
    rospy.loginfo(feedback)
    pass

def takeoff_client():
    client = actionlib.SimpleActionClient('takeoff', TakeoffAction)
    client.wait_for_server()
    rate=rospy.Rate(1)
    goal = TakeoffGoal()
    goal.altitude = 5.0   # Set goal altitude to 5 meters

    client.send_goal(goal,feedback_cb=feedback_callback)

    while not rospy.is_shutdown():
        state = client.get_state()
        
        
        rospy.loginfo("Current State: {}".format(state))
        if state == actionlib.GoalStatus.ACTIVE:
            rospy.loginfo("Takeoff is in progress...")
        elif state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Takeoff succeeded!")
            break
        elif state == actionlib.GoalStatus.PREEMPTED:
            rospy.loginfo("Takeoff was preempted!")
            break
        elif state == actionlib.GoalStatus.ABORTED:
            rospy.loginfo("Takeoff aborted!")
            break
        
        rate.sleep()
    
    result = client.get_result()
    rospy.loginfo("Fuck")
    rospy.loginfo(result)
    return result
    # rospy.loginfo("Task Completed: {}".format(result.altitude))


# def exploration_client

if __name__ == '__main__':
    rospy.init_node('takeoff_client')
    takeoff_action=takeoff_client()
    rospy.loginfo(takeoff_action)

