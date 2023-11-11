#include <iostream>
#include <bits/stdc++.h>
#include <fstream>
#include "lawnmow.h"
#include "calibration.h"
#include "corner_points.h"
#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include "g_planner/point.h"
#include <std_msgs/Bool.h>

using namespace std;

g_planner::point trajectory;
vector<vector<vector<double>>> waypoints;

int surface_index = 0;
int path_index = 0;
float path_length[5] = {0,0,0,0,0};

bool feedback = false;

ros::Publisher pub;



struct Point {
    float x;
    float y;
    float z;
};

void display(vector<vector<double>> waypoint){
    for (auto& x: waypoint){
        for (auto& y: x){
            cout<<y<<" ";
        }
        cout<<"\n";
    }
}

void feedback_cb(const std_msgs::Bool::ConstPtr& fb){
    feedback = fb->data;
    if (feedback){
    
        geometry_msgs::Point point;
        trajectory.points.clear();
        for (int j = 0; j < waypoints[surface_index].size(); j++){
            point.x = waypoints[surface_index][j][0];
            point.y = waypoints[surface_index][j][1];
            point.z = waypoints[surface_index][j][2];
            trajectory.points.push_back(point);
            // my_array[j] = point;
        }
        trajectory.length = path_length[path_index];
        pub.publish(trajectory);
        surface_index++;
        path_index++;
        // for(int k = 0; k < 100; k++){
        //     ros::spinOnce();
        //     loop_rate.sleep();
        // }
    }
}

void publish(){

    geometry_msgs::Point point;
    for (int j = 0; j < waypoints[surface_index].size(); j++){
        point.x = waypoints[surface_index][j][0];
        point.y = waypoints[surface_index][j][1];
        point.z = waypoints[surface_index][j][2];
        trajectory.points.push_back(point);
    }
    trajectory.length = path_length[path_index];
    pub.publish(trajectory);
    surface_index++;
    path_index++;
    // for(int k = 0; k < 50; k++){
    //     pub.publish(trajectory);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
}

 
int main(int argc, char **argv){

    ros::init(argc, argv, "main");
    ros::NodeHandle n;
    ros::Subscriber surface_feedback = n.subscribe<std_msgs::Bool>("surface_feedback",1,feedback_cb);
    pub = n.advertise<g_planner::point>("trajectory", 1000);
    ros::Rate loop_rate(0.5);

    int PPI, pixel_length, pixel_width;
    float fro_ov, sid_ov, FOV;
    double x,y,z;
    ifstream inFile;


    vector<pair<double,pair<double,double>>> arr;


    cout<<"Input PPI range between 70 - 90\n";
    cin>>PPI;
    cout<<"Input the needed resolution in terms of length and width\n";
    cin>>pixel_length>>pixel_width;
    cout<<"Input the Field of View of camera in terms of Degree\n";
    cin>>FOV;
    cout<<"Input Side and Front Overlap Percentage\n";
    cin>>sid_ov>>fro_ov;

    cout<<"Reading Corner Points\n";

    inFile.open("/home/ankit/project_as/src/g_planner/in2.txt");
    if (!inFile) {
        cout << "Unable to open file\n";
        exit(1); // terminate with error
    }
    while (inFile >>x>>y>>z){
        arr.push_back({x, {y,z}});
    }

    inFile.close();

    vector<pair<double,pair<double,double>>> surface_1 = get_surface1(arr);
    vector<pair<double,pair<double,double>>> surface_2 = get_surface2(arr);
    vector<pair<double,pair<double,double>>> surface_3 = get_surface3(arr);
    vector<pair<double,pair<double,double>>> surface_4 = get_surface4(arr);
    vector<pair<double,pair<double,double>>> surface_5 = get_surface5(arr);
    // surface_display(surface_3);
    // surface_display(surface_4);

    // vector<vector<pair<double,pair<double,double>>>> surfaces;
    // surfaces.push_back(surface_1);
    // surfaces.push_back(surface_4);
    // surfaces.push_back(surface_2);
    // surfaces.push_back(surface_3);
    // surfaces.push_back(surface_5);

    cout<<"Calculation is done considering an aspect ratio of 4:3\n";

    double drone_dist = dist_to_drone(PPI,pixel_length,pixel_width,FOV);
    printf("For %d PPI the Distance of the Drone to the Structure must be %lf meters\n",PPI,drone_dist);


    float dimension[2];
    area_captured(PPI,pixel_length,pixel_width,FOV,dimension);

    // dimension[0] is width and dimension[1] is the length of the area seen by the image


    float distX = dimension[0] * (sid_ov/100);
    float distY = dimension[1] * (fro_ov/100);

    bool ascending = true;
    bool neg = true;
    bool mirror_xz = false;
    bool mirror_yz = true;
    bool mirror_xy = false;
    float path_length[5] = {0,0,0,0,0};


    if (ascending and neg){
        waypoints.push_back(find_waypoint(dimension[0],dimension[1], surface_3, distX, distY, ascending, -drone_dist, mirror_xz, path_length[0]));
        neg = false;
        ascending = false;
    }
    if(!ascending and !neg){
        waypoints.push_back(find_waypoint(dimension[0],dimension[1], surface_2, distX, distY, ascending, drone_dist, mirror_yz, path_length[1]));
        ascending = true;
    }
    if(ascending and !neg){
        waypoints.push_back(find_waypoint(dimension[0],dimension[1], surface_4, distX, distY, ascending, drone_dist, mirror_xz, path_length[2]));
        ascending = false;
        neg = true;
    }
    if (!ascending and neg){
        waypoints.push_back(find_waypoint(dimension[0],dimension[1], surface_1, distX, distY, ascending, -drone_dist, mirror_yz, path_length[3]));
        ascending = true;
        neg = false;
    }
    waypoints.push_back(find_waypoint(dimension[0],dimension[1], surface_5, distX, distY, ascending, drone_dist, mirror_xy, path_length[4]));

    publish();

    while (ros::ok()){

        ros::spinOnce();
        loop_rate.sleep();
    }
        
    return 0;
}
