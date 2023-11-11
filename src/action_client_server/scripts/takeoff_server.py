#!/usr/bin/env python

import rospy
import actionlib
from mavros_msgs.srv import CommandBool, SetMode , CommandBoolRequest,SetModeRequest
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from action_client_server.msg import TakeoffAction, TakeoffGoal, TakeoffResult, TakeoffFeedback

class TakeoffServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('takeoff', TakeoffAction, self.execute, False)
        self.server.start()
        self.feedback = TakeoffFeedback()
        self.result = TakeoffResult()

        
        self.setpoint_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.pose_callback)
        self.state_sub = rospy.Subscriber('/mavros/state', State, callback=self.state_callback)

    def state_callback(self, msg):
        self.current_state = msg
        rospy.loginfo('Current mode: {}'.format(msg.mode))

    def pose_callback(self, msg):
        self.current_pose = msg

    def arm(self):
        rospy.loginfo('Arming')
        success = False
        while not success:
            req = CommandBoolRequest()
            req.value = True
            success = self.arm_client.call(req).success
            rospy.loginfo(success)
            rospy.sleep(0.1)


# SEE Why this was not workingggggggggggg
    # def set_mode(self, mode):
    #     rospy.loginfo('Setting mode: {}'.format(mode))
    #     success = False
    #     while not success:
    #         success = self.set_mode_client(custom_mode=mode).mode_sent
            
    #         rospy.sleep(0.1)



    def set_mode(self,mode):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            flight_mode_service(custom_mode=mode)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def execute(self, goal):
        self.set_mode('AUTO.LOITER')
        self.arm()
        
        self.set_mode('OFFBOARD')
        success = True
        goal_altitude = goal.altitude
        rospy.loginfo(goal.altitude)
        rate = rospy.Rate(10)
        while self.current_pose is None:
            rate.sleep()
        takeoff_altitude = self.current_pose.pose.position.z + goal_altitude
        pose=PoseStamped()
        pose.pose.position.x=0
        pose.pose.position.y=0
        pose.pose.position.z=takeoff_altitude
        while self.current_pose.pose.position.z < takeoff_altitude - 0.1:
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                success = False
                break
            self.feedback.reached_altitude = False
            self.server.publish_feedback(self.feedback)
            
            for i in range(50):   
                if(rospy.is_shutdown()):
                    break

                self.setpoint_pub.publish(pose)
            rate.sleep()
        # success=True
        # if success:
        #     rospy.loginfo('Takeoff succeeded')
        #     self.server.set_succeeded()
        if success:
            self.result.success = True
            rospy.loginfo('Takeoff succeeded')
            self.server.set_succeeded(self.result)

if __name__ == '__main__':
    rospy.init_node('takeoff_server')
    server = TakeoffServer()
    rospy.spin()

