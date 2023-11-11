#!/usr/bin/env python



import rospy
from std_msgs.msg import String,Bool

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('uav_2_publisher')
    
    # Wait for 5 minutes
    rospy.sleep(10)
    
    # Create a publisher with the name "rain" and the message type "String"
    pub = rospy.Publisher('uav_2_flag',Bool,queue_size=10)
    
    
    
    count = 0
    while not rospy.is_shutdown() and count < 10:
        pub.publish(True)
        count += 1
        rospy.sleep(0.1) # publish at a rate of 10 Hz
        
    # Print a message indicating that the node is stopping
    rospy.loginfo("Published the message 10 times. Stopping the node.")
    
    # Exit the node
    rospy.signal_shutdown("Published the message 1000 times.")