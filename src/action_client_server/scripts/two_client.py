#!/usr/bin/env python

import rospy
import actionlib
from action_client_server.msg import *
from geometry_msgs.msg import PoseStamped
import datetime


def Rain_cb(msg):
    global raining_flag
    if msg.data=='Yes':
        raining_flag=True
    if msg.data=="No":
        raining_flag=False
        
def feedback_callback_action1(feedback):
    # process feedback here
    rospy.loginfo(feedback)
    pass

def FeedBack_Callback(feedback):
    rospy.loginfo("Distance Left to target = {}".format(feedback))
    pass

def takeoff_client():
    client = actionlib.SimpleActionClient('takeoff', TakeoffAction)
    client.wait_for_server()
    rate=rospy.Rate(1)
    goal = TakeoffGoal()
    goal.altitude = 5.0   # Set goal altitude to 5 meters

    client.send_goal(goal,feedback_cb=feedback_callback_action1)

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
    
    rospy.loginfo(result)
    return result
    


def exploration_client():
    client = actionlib.SimpleActionClient('exploration', ExplorationAction)
    client.wait_for_server()
    rate=rospy.Rate(1)
    goal=ExplorationGoal()
    
    goal.trgt_pose.pose.position.x=20
    goal.trgt_pose.pose.position.y=20
    goal.trgt_pose.pose.position.z=5
    client.send_goal(goal,feedback_cb=FeedBack_Callback)
    while not rospy.is_shutdown():
        state = client.get_state()
        if raining_flag==True:
            client.cancel_all_goals()
            
        
        
        rospy.loginfo("Current State: {}".format(state))
        if state == actionlib.GoalStatus.ACTIVE:
            rospy.loginfo("Moving Towards target position ....")
        elif state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Reached Target Position.....")
            break
        elif state == actionlib.GoalStatus.PREEMPTED:
            rospy.loginfo("Exploration task Preempted!!!!")
            break
        elif state == actionlib.GoalStatus.ABORTED:
            rospy.loginfo("Exploration Task aborted!!!!")
            break
        
        rate.sleep()
    result=client.get_result()
    rospy.loginfo(result)


if __name__ == '__main__':
    rospy.init_node('takeoff_client')
    #Subscribed to raining topic for now to detecting the rain
    rospy.Subscriber('raining',bool,callback=Rain_cb)
    takeoff_action=takeoff_client()
    
    rospy.loginfo('20 seconds Gap')
    rospy.sleep(20)
    exploration_action=exploration_client()
    
        
    

