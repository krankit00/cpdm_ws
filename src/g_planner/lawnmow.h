#include<iostream>
#include<bits/stdc++.h>

using namespace std;

#ifndef LAWNMOW_H
#define LAWNMOW_H
vector<vector<double> > find_waypoint(float width_cap, float len_cap, vector<pair<double,pair<double,double>>> surface,float h_dist,
                                        float v_dist, bool ascending, float drone_dist, bool& mirror, float& path_length);
bool asc_sort_points_xz(const pair<double,pair<double,double>>& a, const pair<double,pair<double,double>>& b);
bool desc_sort_points_xz(const pair<double,pair<double,double>>& a, const pair<double,pair<double,double>>& b);
bool asc_sort_points_xy(const pair<double,pair<double,double>>& a, const pair<double,pair<double,double>>& b);
bool desc_sort_points_xy(const pair<double,pair<double,double>>& a, const pair<double,pair<double,double>>& b);
bool asc_sort_points_yz(const pair<double,pair<double,double>>& a, const pair<double,pair<double,double>>& b);
bool desc_sort_points_yz(const pair<double,pair<double,double>>& a, const pair<double,pair<double,double>>& b);

#endif