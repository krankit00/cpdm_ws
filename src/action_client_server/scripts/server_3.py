#!/usr/bin/env python

import rospy
import actionlib
from mavros_msgs.srv import CommandBool, SetMode , CommandBoolRequest,SetModeRequest,CommandHome,CommandTOL
from mavros_msgs.msg import State,ExtendedState
from geometry_msgs.msg import PoseStamped
from action_client_server.msg import *
import math

class Server:
    def __init__(self):
        self.server_action1 = actionlib.SimpleActionServer('takeoff', TakeoffAction, self.execute_action1, False)
        self.server_actionx = actionlib.SimpleActionServer('takeoff_1', Takeoff_1Action, self.execute_action_x, False)
        self.server_action2 = actionlib.SimpleActionServer('exploration', ExplorationAction, self.execute_action2, False)
        self.server_action3 = actionlib.SimpleActionServer('waypoint_exploration', Waypoint_explorationAction, self.execute_action3, False)
        self.server_action4 = actionlib.SimpleActionServer('trajectory_exploration', TrajectoryAction, self.execute_action4, False)
        self.server_action5 = actionlib.SimpleActionServer('trajectory_exploration_uav1', Trajectory_uav2Action, self.execute_action5, False)
        
        self.server_action1.start()
        self.server_actionx.start()
        self.server_action2.start()
        self.server_action3.start()
        self.server_action4.start()
        self.server_action5.start()
                

        
        self.setpoint_pub_uav0 = rospy.Publisher('uav0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.set_mode_client_uav0 = rospy.ServiceProxy('/uav0/mavros/set_mode', SetMode)
        self.arm_client_uav0 = rospy.ServiceProxy('/uav0/mavros/cmd/arming', CommandBool)
        self.pose_sub_uav0= rospy.Subscriber('/uav0/mavros/local_position/pose', PoseStamped, callback=self.pose_callback_uav0)
        self.state_sub_uav0 = rospy.Subscriber('/uav0/mavros/state', State, callback=self.state_callback_uav0)

        self.setpoint_pub_uav1 = rospy.Publisher('uav1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.set_mode_client_uav1 = rospy.ServiceProxy('/uav1/mavros/set_mode', SetMode)
        self.arm_client_uav1= rospy.ServiceProxy('/uav1/mavros/cmd/arming', CommandBool)
        self.pose_sub_uav1= rospy.Subscriber('/uav1/mavros/local_position/pose', PoseStamped, callback=self.pose_callback_uav1)
        self.state_sub_uav1= rospy.Subscriber('/uav1/mavros/state', State, callback=self.state_callback_uav1)

    def state_callback_uav0(self, msg):
        self.current_state_uav0 = msg
        rospy.loginfo('Current mode of UAV0: {}'.format(msg.mode))
    
    def state_callback_uav1(self, msg):
        self.current_state_uav1= msg
        rospy.loginfo('Current mode of UAV1: {}'.format(msg.mode))

    def pose_callback_uav0(self, msg):
        self.current_pose_uav0 = msg
    def pose_callback_uav1(self, msg):
        self.current_pose_uav1 = msg

    def arm_uav0(self):
        rospy.loginfo('Arming UAV0')
        success = False
        while not success:
            req = CommandBoolRequest()
            req.value = True
            success = self.arm_client_uav0.call(req).success
            rospy.loginfo(success)
            rospy.sleep(0.1)

    def arm_uav1(self):
        rospy.loginfo('Arming UAV1')
        success = False
        while not success:
            req = CommandBoolRequest()
            req.value = True
            success = self.arm_client_uav1.call(req).success
            rospy.loginfo(success)
            rospy.sleep(0.1)


# SEE Why this was not workingggggggggggg
    # def set_mode(self, mode):
    #     rospy.loginfo('Setting mode: {}'.format(mode))
    #     success = False
    #     while not success:
    #         success = self.set_mode_client(custom_mode=mode).mode_sent
            
    #         rospy.sleep(0.1)

    
    def set_mode_uav0(self,mode):
        rospy.wait_for_service('/uav0/mavros/set_mode')
        try:
            flight_mode_service = rospy.ServiceProxy('/uav0/mavros/set_mode', SetMode)
            flight_mode_service(custom_mode=mode)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def set_mode_uav1(self,mode):
        rospy.wait_for_service('/uav1/mavros/set_mode')
        try:
            flight_mode_service = rospy.ServiceProxy('/uav1/mavros/set_mode', SetMode)
            flight_mode_service(custom_mode=mode)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)



    def dist_from_closest_lndng_stn_uav0(self):
        lndng_stn1=PoseStamped()
        lndng_stn1.pose.position.x=-1
        lndng_stn1.pose.position.y=0
        lndng_stn1.pose.position.z=0
        lndng_stn2=PoseStamped()
        lndng_stn2.pose.position.x=-1
        lndng_stn2.pose.position.y=54
        lndng_stn2.pose.position.z=0
        dist1= math.sqrt((self.current_pose_uav0.pose.position.x - lndng_stn1.pose.position.x) ** 2 + (self.current_pose_uav0.pose.position.y - lndng_stn1.pose.position.y) ** 2 + (self.current_pose_uav0.pose.position.z -lndng_stn1.pose.position.z ) ** 2)
        dist2= math.sqrt((self.current_pose_uav0.pose.position.x - lndng_stn2.pose.position.x) ** 2 + (self.current_pose_uav0.pose.position.y - lndng_stn2.pose.position.y) ** 2 + (self.current_pose_uav0.pose.position.z -lndng_stn2.pose.position.z ) ** 2)
        if dist1<=dist2:
            rospy.loginfo("UAV0 Going to Landing Station1")
            return lndng_stn1
        else:
            rospy.loginfo("UAV0 Going to Landing Station 2")
            return lndng_stn2
    
    def dist_from_closest_lndng_stn_uav1(self):
        lndng_stn1=PoseStamped()
        lndng_stn1.pose.position.x=1
        lndng_stn1.pose.position.y=0
        lndng_stn1.pose.position.z=0
        lndng_stn2=PoseStamped()
        lndng_stn2.pose.position.x=1
        lndng_stn2.pose.position.y=54
        lndng_stn2.pose.position.z=0
        dist1= math.sqrt((self.current_pose_uav1.pose.position.x - lndng_stn1.pose.position.x) ** 2 + (self.current_pose_uav1.pose.position.y - lndng_stn1.pose.position.y) ** 2 + (self.current_pose_uav1.pose.position.z -lndng_stn1.pose.position.z ) ** 2)
        dist2= math.sqrt((self.current_pose_uav1.pose.position.x - lndng_stn2.pose.position.x) ** 2 + (self.current_pose_uav1.pose.position.y - lndng_stn2.pose.position.y) ** 2 + (self.current_pose_uav1.pose.position.z -lndng_stn2.pose.position.z ) ** 2)
        if dist1<=dist2:
            rospy.loginfo("UAV1 Going to Landing Station1")
            return lndng_stn1
        else:
            rospy.loginfo("UAV1 Going to Landing Station 2")
            return lndng_stn2

    
    def extended_state_callback(self,data):
        global landed
        if data.landed_state == ExtendedState.LANDED_STATE_ON_GROUND and not landed:
            rospy.loginfo('Drone has landed')
            landed = True
        elif data.landed_state != ExtendedState.LANDED_STATE_ON_GROUND and landed:
            rospy.loginfo('Drone is no longer landed')
        landed = False
    

    def has_reached_desired_position_uav0(self,desired_pose):
        pose=PoseStamped()
        pose=desired_pose
        x1 = self.current_pose_uav0.pose.position.x
        y1 = self.current_pose_uav0.pose.position.y
        z1 = self.current_pose_uav0.pose.position.z

        x2 = pose.pose.position.x
        y2 = pose.pose.position.y
        z2 = pose.pose.position.z

        distance = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)
        # rospy.loginfo(distance)
        if distance <= 2:
            return True
        else:
            return False
    def has_reached_desired_position_uav1(self,desired_pose):
        pose=PoseStamped()
        pose=desired_pose
        x1 = self.current_pose_uav1.pose.position.x
        y1 = self.current_pose_uav1.pose.position.y
        z1 = self.current_pose_uav1.pose.position.z

        x2 = pose.pose.position.x
        y2 = pose.pose.position.y
        z2 = pose.pose.position.z

        distance = math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)
        # rospy.loginfo(distance)
        if distance <= 2:
            return True
        else:
            return False
    
    def percentage_covered(self,total_pts,pts_covered):
        value=(pts_covered/total_pts)*100
        return value


    def execute_action1(self, goal):
        self.feedback = TakeoffFeedback()
        self.result = TakeoffResult()
        self.set_mode_uav0('AUTO.LOITER')
        self.arm_uav0()

        self.set_mode_uav0('OFFBOARD')
        success = True
        goal_altitude = goal.altitude
        rate = rospy.Rate(10)
        while self.current_pose_uav0 is None:
            rate.sleep()
        takeoff_altitude = goal_altitude

        pose=PoseStamped()
        pose.pose.position.x=self.current_pose_uav0.pose.position.x
        pose.pose.position.y=self.current_pose_uav0.pose.position.y
        pose.pose.position.z=takeoff_altitude
        while self.current_pose_uav0.pose.position.z < takeoff_altitude - 0.1:
            if self.server_action1.is_preempt_requested():
                
                self.server_action1.set_preempted()
                success = False
                break
            self.feedback.reached_altitude = False
            self.server_action1.publish_feedback(self.feedback)
            
            for i in range(50):   
                if(rospy.is_shutdown()):
                    break

                self.setpoint_pub_uav0.publish(pose)
            rate.sleep()
        # success=True
        # if success:
        #     rospy.loginfo('Takeoff succeeded')
        #     self.server.set_succeeded()
        if success:
            self.result.success = True
            rospy.loginfo('Takeoff succeeded')
            self.server_action1.set_succeeded(self.result)

    def execute_action_x(self, goal):
        print("INSIDE X FUNCTION")
        self.feedback = Takeoff_1Feedback()
        self.result = Takeoff_1Result()
        self.set_mode_uav1('OFFBOARD')
        self.arm_uav1()
        # self.set_mode_uav1('AUTO.LOITER')
        # self.set_mode_uav1('OFFBOARD')
        success = True
        goal_altitude = goal.altitude
        rate = rospy.Rate(10)
        while self.current_pose_uav1 is None:
            rate.sleep()
        takeoff_altitude = goal_altitude

        pose=PoseStamped()
        pose.pose.position.x=self.current_pose_uav1.pose.position.x
        pose.pose.position.y=self.current_pose_uav1.pose.position.y
        pose.pose.position.z=takeoff_altitude
        while self.current_pose_uav1.pose.position.z < takeoff_altitude - 0.1:
            if self.server_action1.is_preempt_requested():
                
                self.server_actionx.set_preempted()
                success = False
                break
            self.feedback.reached_altitude = False
            self.server_actionx.publish_feedback(self.feedback)
            
            for i in range(50):   
                if(rospy.is_shutdown()):
                    break

                self.setpoint_pub_uav1.publish(pose)
            rate.sleep()
        # success=True
        # if success:
        #     rospy.loginfo('Takeoff succeeded')
        #     self.server.set_succeeded()
        if success:
            self.result.success = True
            rospy.loginfo('Takeoff succeeded')
            self.server_actionx.set_succeeded(self.result)

    def execute_action2(self,goal):
        rospy.loginfo(type(goal))
        rate=rospy.Rate(10)
        lis_wp=[] #List of tuple of setpoints
        self.feedback = ExplorationFeedback()
        self.result = ExplorationResult()
        self.set_mode('AUTO.LOITER')
        self.arm()
        # self.set_mode('AUTO.LOITER')
        self.set_mode('OFFBOARD')
        self.feedback=ExplorationFeedback()
        success=True
        
        
        var=PoseStamped()
        var=goal.trgt_pose
        x1 = var.pose.position.x
        y1 = var.pose.position.y
        z1 = var.pose.position.z

        x2 = self.current_pose.pose.position.x
        y2 = self.current_pose.pose.position.y
        z2 = self.current_pose.pose.position.z

        dist = int(math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2))

        self.feedback.distance_left=dist
        self.server_action2.publish_feedback(self.feedback)
        while dist > 3:
            x2 = self.current_pose.pose.position.x
            y2 = self.current_pose.pose.position.y
            z2 = self.current_pose.pose.position.z
            if self.server_action2.is_preempt_requested():
                set_home_position = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)

                home_pose = PoseStamped()
                
                response = set_home_position(0, 0, 0, home_pose)
                home_pose=self.dist_from_closest_lndng_stn()
                home_pose.pose.position.z=home_pose.pose.position.z 
                self.set_mode("RTL")
                # for i in range(50):
                #     self.setpoint_pub.publish(home_pose)
                rospy.Subscriber('/mavros/extended_state', ExtendedState, callback=self.extended_state_callback)
                self.server_action2.set_preempted()
                success = False
                break
            dist = int(math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2))
            self.feedback.distance_left = dist
            self.server_action2.publish_feedback(self.feedback)
            
            for i in range(50):   
                if(rospy.is_shutdown()):
                    break

                self.setpoint_pub.publish(var)
            rate.sleep()
        rospy.loginfo("HERE")
        if success==False:
            rospy.loginfo("Task was preempted")
        if success==True:
            self.result.success=True
            rospy.loginfo("Reached Destination")
            self.server_action2.set_succeeded(self.result)
    

    def execute_action3(self,goal):
        
        rate=rospy.Rate(20)
        lis_wp=[(0,0,14),(20,0,14),(20,50,14),(-10,50,14),(-10,0,14)] #List of tuple of setpoints
        
        
        self.set_mode('AUTO.LOITER')
        self.arm()
        # self.set_mode('AUTO.LOITER')
        self.set_mode('OFFBOARD')
        self.feedback=Waypoint_explorationFeedback()
        self.result = Waypoint_explorationResult()
        success=True
        
        
        l=0
        k=goal.strtng_pt
        print(k)
        for x,y,z in lis_wp[k:]:
            self.set_mode('OFFBOARD')
            pose=PoseStamped()
            pose.pose.position.x=x
            pose.pose.position.y=y
            pose.pose.position.z=z
            rospy.loginfo(pose.pose.position.x)
            if self.server_action3.is_preempt_requested():
               
                home_pose = PoseStamped()
                
                
                home_pose=self.dist_from_closest_lndng_stn()
                home_pose.pose.position.z=home_pose.pose.position.z + 10
                for i in range(100):
                    self.setpoint_pub.publish(home_pose)
                    rate.sleep()
                # response = set_home_position(0, 0, 0, home_pose)
                rospy.wait_for_service('/mavros/cmd/land')

                # Create a service proxy for the command service
                land_service = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

                # Call the service with the landing altitude (in meters)
                landing_altitude = 0.0 # Set to 0.0 to land on the current position
               
                response = land_service(altitude=landing_altitude)
                # self.set_mode("RTL")
                self.result.no_points_covered = l 
                # rospy.Subscriber('/mavros/extended_state', ExtendedState, callback=self.extended_state_callback)
                self.server_action3.set_preempted(self.result)
                success = False
                break

            self.feedback.no_points_covered = l 
            self.server_action3.publish_feedback(self.feedback)
            rate=rospy.Rate(7)
            for i in range(100):   
                if(rospy.is_shutdown()):
                    break
                # rospy.loginfo("Publishing SetPoints")
                self.setpoint_pub.publish(pose)
                rate.sleep()
            l+=1
            # rate=rospy.Rate(20)
            while  not self.has_reached_desired_position(pose):
                rospy.sleep(1)
                rospy.loginfo("Waiting for drone to reach {}th waypoint.".format(l))
                
            # rospy.sleep(2)
            # rate.sleep()
            rospy.loginfo("Drone reached {}th point".format(l))
        
        if success==False:
            self.result.no_points_covered = l
            
            rospy.loginfo("Task was preempted")
        if success==True:
            self.result.success=True
            self.result.no_points_covered=l
            rospy.loginfo("Reached Destination")
            self.server_action3.set_succeeded(self.result)
    
    def execute_action4(self,goal):
        print("Inside _Execute 4")
        lis_wp=goal.traj_points
        # self.set_mode('AUTO.LOITER')
        self.set_mode_uav0('OFFBOARD')
        self.arm_uav0()
        # self.set_mode('AUTO.LOITER')
        
        self.feedback=TrajectoryFeedback()
        self.result = TrajectoryResult()
        success=True
        
        
        
        k=goal.starting_point
        l=k
      

        
        for i in range(k,len(lis_wp)):
        
            self.set_mode_uav0('OFFBOARD')
            pose=PoseStamped()
            pose.pose.position.x=lis_wp[i].x
            pose.pose.position.y=lis_wp[i].y

            pose.pose.position.z=lis_wp[i].z
            
            if self.server_action4.is_preempt_requested():
                #Note ::::::::::::::::Rate is of utmost importance here previously rate was 200
                #  hz for outer looop due to that drone was not rising to mentioned altitude and
                #  was not hovering exactly over landing station but after decaresing the rate to 10 it behaves perfectly

                rate=rospy.Rate(10)
                #Takeoff to obstacle free altitude since no local planner right now
                takeoff_pose=PoseStamped()
                takeoff_pose.pose.position.x=self.current_pose_uav0.pose.position.x
                takeoff_pose.pose.position.y=self.current_pose_uav0.pose.position.y
                takeoff_pose.pose.position.z=17
                for i in range(100):
                    self.setpoint_pub_uav0.publish(takeoff_pose)
                    rate.sleep()
                rospy.sleep(2)


                home_pose = PoseStamped()
                home_pose=self.dist_from_closest_lndng_stn_uav0()
                home_pose.pose.position.z=home_pose.pose.position.z + 14
                for i in range(100):
                    self.setpoint_pub_uav0.publish(home_pose)
                    rate.sleep()

                while not self.has_reached_desired_position_uav0(home_pose):
                    rospy.sleep(1.5)
                    print("Waiting for drone to hover over landing station")
                    pass
                # response = set_home_position(0, 0, 0, home_pose)
                rospy.wait_for_service('/uav0/mavros/cmd/land')

                # Create a service proxy for the command service
                land_service = rospy.ServiceProxy('/uav0/mavros/cmd/land', CommandTOL)

                # Call the service with the landing altitude (in meters)
                landing_altitude = 0.0 # Set to 0.0 to land on the current position
                
                response = land_service(altitude=landing_altitude)
                
                self.result.points_covered = l 
                # rospy.Subscriber('/mavros/extended_state', ExtendedState, callback=self.extended_state_callback)
                self.server_action4.set_preempted(self.result)
                success = False
                break
            self.feedback.no_points_covered=l
            self.feedback.percentage = self.percentage_covered(len(lis_wp),l) 
            self.server_action4.publish_feedback(self.feedback)
            rate=rospy.Rate(200)
            for i in range(100):   
                if(rospy.is_shutdown()):
                    break
                # rospy.loginfo("Publishing SetPoints")
                self.setpoint_pub_uav0.publish(pose)
                rate.sleep()
            l+=1
            # rate=rospy.Rate(20)
            # while  not self.has_reached_desired_position(pose):
            #     # rospy.sleep(0.01)
            #     # rospy.loginfo("Waiting for drone to reach {}th waypoint.".format(l))
            #     pass
                
            # rospy.sleep(2)
            # rate.sleep()
            # rospy.loginfo("Drone reached {}th point".format(l))
        
        if success==False:
            self.result.surface_success=False
            self.result.points_covered=l
            
            rospy.loginfo("Task was preempted")
        if success==True:
            self.result.surface_success=True
            
            rospy.loginfo("Reached Destination")
            self.server_action4.set_succeeded(self.result)


    def execute_action5(self,goal):
        print("Inside _Execute 5")
        lis_wp=goal.traj_points
        # self.set_mode('AUTO.LOITER')
        self.set_mode_uav1('OFFBOARD')
        self.arm_uav1()
        # self.set_mode('AUTO.LOITER')
        
        self.feedback=Trajectory_uav2Feedback()
        self.result = Trajectory_uav2Result()
        success=True
        
        
        
        k=goal.starting_point
        l=k
        print(self.current_pose_uav1.pose.position.y)
        print(lis_wp[k].y)
        pose=PoseStamped()
        pose.pose.position.x=lis_wp[k].x-1
        pose.pose.position.y=lis_wp[k].y
        pose.pose.position.z=17
        rate=rospy.Rate(15)
        for i in range(50):
            self.setpoint_pub_uav1.publish(pose)
            rate.sleep()

        
        for i in range(k,len(lis_wp)):
        
            self.set_mode_uav1('OFFBOARD')
            pose=PoseStamped()
            pose.pose.position.x=lis_wp[i].x-1
            pose.pose.position.y=lis_wp[i].y
            pose.pose.position.z=lis_wp[i].z
            

            if self.server_action5.is_preempt_requested():
                #Note ::::::::::::::::Rate is of utmost importance here previously rate was 200
                #  hz for outer looop due to that drone was not rising to mentioned altitude and
                #  was not hovering exactly over landing station but after decaresing the rate to 10 it behaves perfectly

                rate=rospy.Rate(10)
                #Takeoff to obstacle free altitude since no local planner right now
                takeoff_pose=PoseStamped()
                takeoff_pose.pose.position.x=self.current_pose_uav1.pose.position.x
                takeoff_pose.pose.position.y=self.current_pose_uav1.pose.position.y
                takeoff_pose.pose.position.z=17
                for i in range(100):
                    self.setpoint_pub_uav1.publish(takeoff_pose)
                    rate.sleep()
                rospy.sleep(3)


                home_pose = PoseStamped()
                home_pose=self.dist_from_closest_lndng_stn_uav1()
                home_pose.pose.position.z=home_pose.pose.position.z + 14
                for i in range(100):
                    self.setpoint_pub_uav1.publish(home_pose)
                    rate.sleep()

                while not self.has_reached_desired_position_uav1(home_pose):
                    rospy.sleep(1.5)
                    print("Waiting for drone to hover over landing station")
                    pass
                # response = set_home_position(0, 0, 0, home_pose)
                rospy.wait_for_service('/uav1/mavros/cmd/land')

                # Create a service proxy for the command service
                land_service = rospy.ServiceProxy('/uav1/mavros/cmd/land', CommandTOL)

                # Call the service with the landing altitude (in meters)
                landing_altitude = 0.0 # Set to 0.0 to land on the current position
                
                response = land_service(altitude=landing_altitude)
                
                self.result.points_covered = l 
                # rospy.Subscriber('/mavros/extended_state', ExtendedState, callback=self.extended_state_callback)
                self.server_action5.set_preempted(self.result)
                success = False
                break
            self.feedback.no_points_covered=l
            self.feedback.percentage = self.percentage_covered(len(lis_wp),l) 
            self.server_action5.publish_feedback(self.feedback)
            rate=rospy.Rate(200)
            for i in range(100):   
                if(rospy.is_shutdown()):
                    break
                # rospy.loginfo("Publishing SetPoints")
                self.setpoint_pub_uav1.publish(pose)
                rate.sleep()
            l+=1
            # rate=rospy.Rate(20)
            # while  not self.has_reached_desired_position(pose):
            #     # rospy.sleep(0.01)
            #     # rospy.loginfo("Waiting for drone to reach {}th waypoint.".format(l))
            #     pass
                
            # rospy.sleep(2)
            # rate.sleep()
            # rospy.loginfo("Drone reached {}th point".format(l))
        
        if success==False:
            self.result.surface_success=False
            self.result.points_covered=l
            
            rospy.loginfo("Task was preempted")
        if success==True:
            self.result.surface_success=True
            
            rospy.loginfo("Reached Destination")
            self.server_action5.set_succeeded(self.result)

if __name__ == '__main__':
    rospy.init_node('takeoff_server')
    server = Server()
    rospy.spin()

