#!/usr/bin/env python

import rospy
import actionlib
from action_client_server.msg import *
from geometry_msgs.msg import PoseStamped
import datetime
from std_msgs.msg import Bool
from g_planner.msg import point

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

def Trajectory_feedback(feedback):
    rospy.loginfo(feedback)

send_traj=False
def traj_cb(msg):
    global send_traj

    print("Inside Callback")
    global list_pts
    print(type(msg))
    list_pts=msg.points
    # list(list_pts)
    print("Points Received")
    
    
    trajectory()
    # send_traj=True



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

def waypoint_exploration_client(k):
    client = actionlib.SimpleActionClient('waypoint_exploration', Waypoint_explorationAction)
    client.wait_for_server()
    rate=rospy.Rate(1)
    goal=Waypoint_explorationGoal()
    goal.strtng_pt=k
    client.send_goal(goal,feedback_cb=Waypoint_exploration_FeedBack_Callback)
    while not rospy.is_shutdown():
        state = client.get_state()
        rospy.loginfo(raining_flag)
        if raining_flag==True:
            rospy.loginfo(raining_flag)
            client.cancel_goal()
            
        
        
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
    print(type(result))
    print(result.no_points_covered)
    return result.no_points_covered ,result.success
    
    rospy.loginfo(result)

def trajectory():
    client = actionlib.SimpleActionClient('trajectory_exploration', TrajectoryAction)
    client.wait_for_server()
    rate=rospy.Rate(1)
    goal=TrajectoryGoal()
    goal.traj_points=list_pts
    k=0
    surface_success=False
    while not surface_success:
        if raining_flag==True:
            pass
        if raining_flag==False:
            takeoff_client()
            goal.starting_point=k
            client.send_goal(goal,feedback_cb=Trajectory_feedback)

            while not rospy.is_shutdown():
                state = client.get_state()
                rospy.loginfo(raining_flag)
                if raining_flag==True:
                    # rospy.loginfo(raining_flag)
                    client.cancel_goal()
                    
                
                
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
        surface_success=result.surface_success
        k=result.points_covered    
    if result.surface_success==True:
        x=True
        surf_pub.publish(x)
        
    


    
    


if __name__ == '__main__':
    rospy.init_node('takeoff_client')
    #Subscribed to raining topic for now to detect the rain
    rospy.Subscriber('rain',Bool,callback=Rain_cb)
    rospy.Subscriber('trajectory',point,queue_size=1,callback=traj_cb)
    surf_pub=rospy.Publisher('surface_feedback',Bool,queue_size=1)
    takeoff_client()
    
    rospy.spin()
    
    
    
        
    

