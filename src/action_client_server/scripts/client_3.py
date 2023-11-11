#!/usr/bin/env python

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

raining_flag=False

def acquire_weather_data():
    

    # Set your OpenWeatherMap API key
    api_key = 'a87562299d69d234e17dc4541ee51b07'

    # Set the location for which you want to get the weather data
    print('''
Instructions to find Latitude and Longitude :
1.To know latitude and longitude go to maps application 
2.Long press your loaction icon(Blue circle)"
3.You will find latitude(N) and longitude(E)
             
             
             ''')
    latitude=float(input("Enter latitude (N) of your location : "))
    print()
    longitude=float(input('Enter longitude (E) of you location : '))
    print()
    
    response = requests.get(f'http://api.openweathermap.org/data/2.5/weather?lat={latitude}&lon={longitude}&appid={api_key}')

    
    weather_data = response.json()
    weather_condition = weather_data['weather'][0]['main']

    
    if weather_condition == 'Clear':
        print(f"It's clear at the site.")

    elif weather_condition == 'Clouds':
        print(f"It's cloudy at the site.")

    elif weather_condition == 'Rain':
        print(f"It's raining on the site.")

    return weather_condition


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
global uav_2_flag
uav_2_flag=False

def Uav2(msg):
    global uav_2_flag

    if msg.data==True:
        uav_2_flag=True
        print("UAV 2 flag is {}".format(uav_2_flag))
    if msg.data==False:
        uav_2_flag=False

def traj_cb(msg):
    global send_traj
    global uav_2_flag
    
    print("Inside Callback")
    global list_pts
    print(type(msg))
    list_pts=msg.points
    # list(list_pts)
    print("Points Received")
    a_l=[]
    # count=0
    # while count<6:
    #     lst=[msg.points[count].x,msg.points[count].y,msg.points[count].z]
    #     a_l.append(lst)
    # print(a_l)
    print(list_pts)

    if uav_2_flag==False:
          
        uav_2_flag,k,a=trajectory()
        a=a.lower()
        if a=="yes":
            trajectory_uav1(k)      
        if a=="no":
            print("Mission Aborted")     
    if uav_2_flag:
        trajectory_uav1(0)

    


def takeoff_client_0():
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

def takeoff_client_1():
    client = actionlib.SimpleActionClient('takeoff_1', Takeoff_1Action)
    client.wait_for_server()
    rate=rospy.Rate(20)
    goal = Takeoff_1Goal()
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
            takeoff_client_0()
            goal.starting_point=k
            client.send_goal(goal,feedback_cb=Trajectory_feedback)
            
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
    return uav_2_flag,k,a
        
def trajectory_uav1(t):
    client = actionlib.SimpleActionClient('trajectory_exploration_uav1', Trajectory_uav2Action)
    client.wait_for_server()
    rate=rospy.Rate(1)
    goal=Trajectory_uav2Goal()
    goal.traj_points=list_pts
    k=t
    surface_success=False
    
    while not surface_success:
        if raining_flag==True:
            pass
        if raining_flag==False:
            takeoff_client_1()
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
    
    uav_2_flag=False
    

    #Subscribers and Publishersto different topics
    rospy.Subscriber('rain',Bool,callback=Rain_cb)
    rospy.Subscriber('trajectory',point,queue_size=1,callback=traj_cb)
    rospy.Subscriber('uav_2_flag',Bool,callback=Uav2)
    surf_pub=rospy.Publisher('surface_feedback',Bool,queue_size=1)
    
    #Condition for checking time
    current_time = datetime.datetime.now().time()
    # while True:
    #     if current_time.hour == 21 and current_time.minute == 0:
    #         break
    print('''
    
    ''')
    print("Its 9 A.M. . So time for executing the mission")
    print()
    # weather_condition=acquire_weather_data()
    # while True:
    #     if weather_condition=='Clear':
    #         print("We are expexting clear weather conditions")
    #         break
    #     if weather_condition=="Clouds":
    #         print("We are expecting cloudy conditions")
    #         break
    #     if weather_condition=='Rain':
    #         print("It's raining,so mission needs to be postponed")
    #         rospy.sleep(60)
        

    #Checking for operator inputs
    while True:
        print()
        operator_input=input("Should we go on with the mission (Yes) or (No) : ")
        operator_input=operator_input.lower()
        if operator_input=='yes':
            break
        if operator_input=='no':
            print()
            print("Postponing for 10 minutes")
        rospy.sleep(600)
    

    #Cheking for customer needs
    print()
    print()
    user_need=input("What type of Result is needed (Detailed View (press 1)) or (Bird Eye View (press 2)) : ")
    if user_need=="1":
        print()
        print()
        print(" You are all set for a Detailed view of the site")
        print()
        print("-----------------------------------------------------------------------------------")
        node_name = "point_publisher"
        package_name = "g_planner"
        # Run the node using rosrun command
        subprocess.Popen(["rosrun", package_name, node_name])
    if user_need=="2":
        print()
        print()
        print("You are all set for the bird eye view")
        print()
        print("-----------------------------------------------------------------------------------")
        node_name="Point_generator.py"
        package_name="birdeye_g_planner"
        subprocess.Popen(["rosrun", package_name, node_name])
            
    # takeoff_client_0()
    # takeoff_client_1()
    
    rospy.spin()
    
    
    
        
    

