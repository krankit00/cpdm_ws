#!/usr/bin/env python

import os
import rospy
from geometry_msgs.msg import PoseStamped, Point
from g_planner.msg import point
import matplotlib.pyplot as plt


if __name__ == '__main__':
    rospy.init_node('points_generator')
    pub = rospy.Publisher('trajectory', point, queue_size=10)
    # get the current directory
    current_directory = os.getcwd()

    # join the file name with the directory path
    file_path = '/home/impact4impact/catkin_ws/src/birdeye_g_planner/scripts/point.txt'


    with open(file_path, 'r') as f:
        # read each line in the file
        lst=[]
        for line in f:
            ls=[]
            # split the line into a list of values
            values = line.strip().split()
            for i in range(len(values)):
                values[i]=float(values[i])
                
            lst.append(values)

    modi_lst=[]
    max_z=0
    max_x=lst[0][0]
    # print(max_x)
    max_y=lst[0][1]
    # print(max_y)
    for i in range(len(lst)):
        if float(lst[i][2])>max_z or float(lst[i][2])==max_z:
            max_z=float(lst[i][2])
            if lst[i][0]>max_x:
                max_x=lst[i][0]
            if lst[i][1]>max_y:
                max_y=lst[i][1]

            modi_lst.append(lst[i])
    # print(max_x)
    # print(max_y)
    # print(modi_lst)       
    for i in range(len(modi_lst)):
        # print(i)
        if modi_lst[i][0]==max_x:
            modi_lst[i][0]+=4
        if modi_lst[i][0]<=max_x:
            modi_lst[i][0]-=4
        if modi_lst[i][1]==max_y:
            modi_lst[i][1]+=4
        if modi_lst[i][1]<=max_y:
            modi_lst[i][1]-=4
        modi_lst[i][2]+=8

    max_z=modi_lst[1][2]   
            
    # Define the four corner points of the rectangle
    # print(modi_lst)
    point1 = modi_lst[1]
    point2 = modi_lst[0]
    point3 = modi_lst[3]
    point4 = modi_lst[2]



    num_step_x=int((abs(point1[0]-point2[0]))/0.5)
    num_step_y=int((abs(point2[1]-point3[1]))/0.5)

    # print(num_step_x)
    # print(num_step_y)

    curx=point1[0]
    cury=point1[1]
    step_y=0
    coordinates=[[curx,cury,max_z]]
    while step_y!=num_step_y:
        for j in range(num_step_y):
            if j%2==0:
                # print("{} line, so left to right".format(j))
                for i in range(num_step_x):
                    curx+=0.5
                    coordinates.append([curx,cury,max_z])
            
            if j%2!=0:
                # print("{} line, so right to left".format(j))
                for i in range(num_step_x):
                    curx-=0.5
                    coordinates.append([curx,cury,max_z])
            
            cury+=1
            coordinates.append([curx,cury,max_z])
            step_y+=1

    # x=[coordinates[i][0] for i in range(len(coordinates))]
    # y=[coordinates[i][1] for i in range(len(coordinates))]
    # plt.plot(x, y)
    # plt.xlabel('x')
    # plt.ylabel('y')
    # plt.title('x vs y')
    # plt.show()

    trajectory=point()
    for coord in coordinates:
        pointi = Point()
        pointi.x = coord[0]
        pointi.y = coord[1]
        pointi.z = coord[2]
        trajectory.points.append(pointi) 
    
    # print(trajectory)
    rate = rospy.Rate(10)
    for i in range(2):
        pub.publish(trajectory)
        rate.sleep()
    rospy.spin()