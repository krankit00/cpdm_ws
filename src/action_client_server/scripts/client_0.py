#!/usr/bin/env python

import rospy
import actionlib
from action_client_server.msg import *
from geometry_msgs.msg import PoseStamped
import datetime
from std_msgs.msg import Bool

raining_flag=False
def Rain_cb(msg):
    global raining_flag
    raining_flag=False
    if msg.data==True:
        raining_flag=True
    if msg.data=="No":
        raining_flag=False
        
def feedback_callback_action1(feedback):
    # process feedback here
    rospy.loginfo(feedback)
    pass

def FeedBack_Callback(feedback):
    rospy.loginfo(feedback)
    pass

def Waypoint_exploration_FeedBack_Callback(feedback):
    rospy.loginfo(feedback)
    pass
    

def takeoff_client():
    client = actionlib.SimpleActionClient('takeoff', TakeoffAction)
    client.wait_for_server()
    rate=rospy.Rate(1)
    goal = TakeoffGoal()
    goal.altitude = 14.0   # Set goal altitude to 5 meters

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

def waypoint_exploration_client():
    client = actionlib.SimpleActionClient('waypoint_exploration', Waypoint_explorationAction)
    client.wait_for_server()
    rate=rospy.Rate(1)
    goal=Waypoint_explorationGoal()
    k=0
    success=False
    while not success:
        if raining_flag==True:
            pass
        if raining_flag==False:
            takeoff_client()
            goal.strtng_pt=k
            client.send_goal(goal,feedback_cb=Waypoint_exploration_FeedBack_Callback)
            while not rospy.is_shutdown():

                state = client.get_state()
                # rospy.loginfo(raining_flag)
                if raining_flag==True:
                    rospy.loginfo(raining_flag)
                    client.cancel_goal()
                    flag=False
                    
                
                
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
            print(result)
            k=result.no_points_covered
            success=result.success
        
    # print(result)
    # return result.no_points_covered ,result.success 
        




if __name__ == '__main__':
    rospy.init_node('takeoff_client')
    #Subscribed to raining topic for now to detect the rain
    rospy.Subscriber('rain',Bool,callback=Rain_cb)
    
    # takeoff_action=takeoff_client()
    # rospy.loginfo('10 seconds Gap')
    # rospy.sleep(10)
    waypoint_exploration_client()

    # k=0
    # status=False
    # # while not status:
    #     takeoff_client()
    #     waypoint_exploration_action,success=waypoint_exploration_client(k)
    #     k=waypoint_exploration_action
    #     rospy.sleep(60)

    print("Exploration Complete")
    
    
        
    

