#!/usr/bin/env python


from transitions import Machine
import actionlib

import rospy
import actionlib
from action_client_server.msg import *
from geometry_msgs.msg import PoseStamped
import datetime
from std_msgs.msg import Bool
from g_planner.msg import point
import subprocess
import datetime
import requests


global raining_flag
raining_flag=False

global uav_2_flag
uav_2_flag=False



def Rain_cb(msg):
    global raining_flag
    raining_flag=False
    if msg.data==True:
        raining_flag=True
    if msg.data=="No":
        raining_flag=False

def traj_cb(msg):
    global send_traj
    global uav_2_flag
    
    print("Inside Callback")
    global list_pts
    print(type(msg))
    list_pts=msg.points
    print("Points Received")
    print(list_pts)

    if uav_2_flag==False:
          
        uav_2_flag,k,a=drone1.trajectory()
    #     a=a.lower()
    #     if a=="yes":
    #         trajectory_uav1(k)      
    #     if a=="no":
    #         print("Mission Aborted")     
    # if uav_2_flag:
    #     trajectory_uav1(0)

class Drone1:
    states = ['landed', 'takeoff', 'exploration', 'hovering']
    transitions = [
        {'trigger': 'takeoff', 'source': 'landed', 'dest': 'takeoff'},
        {'trigger': 'explore', 'source': 'takeoff', 'dest': 'exploration'},
        {'trigger': 'hover', 'source': 'exploration', 'dest': 'hovering'},
        {'trigger': 'land', 'source': ['hovering','exploration'], 'dest': 'landed'}
    ]
    
    def __init__(self, name):
        self.name = name
        
        self.machine = Machine(model=self, states=Drone1.states, transitions=Drone1.transitions, initial='landed')
        self.state = self.machine.initial
    
    def feedback_callback_action1(self,feedback):
        # process feedback here
        rospy.loginfo(feedback)
        pass
    
    def Trajectory_feedback(self,feedback):
        rospy.loginfo(feedback)

    def trigger_transition(self, trigger):
        getattr(self.machine, trigger())


    def takeoff_client_0(self):
        client = actionlib.SimpleActionClient('takeoff', TakeoffAction)
        client.wait_for_server()
        rate=rospy.Rate(1)
        goal = TakeoffGoal()
        goal.altitude = 14.0   # Set goal altitude to 5 meters

        client.send_goal(goal,feedback_cb=self.feedback_callback_action1)

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
    

    def after_takeoff_transition(self):
        print("Drone has taken off")


    def takeoff_drone1(self):
        print(self.state)
        result=self.takeoff_client_0()
        self.machine._get_trigger(model=self,trigger_name='takeoff')
        # if result==True:
        self.after_takeoff_transition()
        print(self.state)
        
    # # Get the trigger function for the 'takeoff' transition
    #     takeoff_trigger = self.machine._get_trigger(model=self,trigger_name='takeoff')
    #     takeoff_trigger
    #     # Decorate the function to execute before and after the transition
    #     @takeoff_trigger
    #     def takeoff_drone1():
    #         result=self.takeoff_client_0()
    #         print(result)
        # Call the decorated function to trigger the transition
        # takeoff_drone1()
    
    
    def trajectory(self):
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
                self.takeoff_client_0()
                goal.starting_point=k
                client.send_goal(goal,feedback_cb=self.Trajectory_feedback)
                
                while not rospy.is_shutdown():
                    state = client.get_state()
                    rospy.loginfo(raining_flag)
                    print("UAV_2_Flag is {}".format(uav_2_flag))
                    if raining_flag==True or uav_2_flag==True:
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
                        if raining_flag==True:
                            print("Due to rainy conditions task was preempted")
                            self.machine._get_trigger(model=self,trigger_name='takeoff')
                        if uav_2_flag==True:
                            print("Due to some issue in drone 1 task was preempted")
                            a=input("Should the second drone be sent (YES or NO):")

                        break
                    elif state == actionlib.GoalStatus.ABORTED:
                        rospy.loginfo("Exploration Task aborted!!!!")
                        break
                    rate.sleep()
                
                result=client.get_result()
            
            surface_success=result.surface_success
            k=result.points_covered    
            if uav_2_flag==True:
                break
        if result.surface_success==True:
            x=True
            surf_pub.publish(x)
        return uav_2_flag,k,a,raining_flag
          

    def explore_drone1(self):
        print(self.state)
        result=self.takeoff_client_0()
        self.machine._get_trigger(model=self,trigger_name='explore')
        print(self.state)
    
if __name__ == '__main__':
    
    # Initial weather acquision and bird eye view detailed view explorationAction
    # done 
    # done 
    rospy.init_node("State_machine")
    rospy.Subscriber('rain',Bool,callback=Rain_cb)
    rospy.Subscriber('trajectory',point,queue_size=1,callback=traj_cb)
    surf_pub=rospy.Publisher('surface_feedback',Bool,queue_size=1)
    drone1=Drone1('Drone 1')
    drone1.takeoff_drone1()
    # rospy.sleep(10)
    # drone1.explore_drone1()
    rospy.spin()
    # print(drone1.state)
    # drone1.takeoff()
    # print(drone1.state)
    # drone1.explore()
    # print(drone1.state)