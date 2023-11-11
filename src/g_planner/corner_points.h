#include<bits/stdc++.h>
#include<iostream>
using namespace std;
#ifndef CORNER_POINTS_H
#define CORNER_POINTS_H
bool sort_surface_xy(const pair<double,pair<double,double>> &a, pair<double,pair<double,double>> &b);
bool sort_surface_yz(const pair<double,pair<double,double>> &a, pair<double,pair<double,double>> &b);
bool sort_surface_xz(const pair<double,pair<double,double>> &a, pair<double,pair<double,double>> &b);
void surface_display(vector<pair<double,pair<double,double>>>& arr);
vector<pair<double,pair<double,double>>> get_surface1(vector<pair<double,pair<double,double>>> arr);
vector<pair<double,pair<double,double>>> get_surface2(vector<pair<double,pair<double,double>>> arr);
vector<pair<double,pair<double,double>>> get_surface3(vector<pair<double,pair<double,double>>> arr);
vector<pair<double,pair<double,double>>> get_surface4(vector<pair<double,pair<double,double>>> arr);
vector<pair<double,pair<double,double>>> get_surface5(vector<pair<double,pair<double,double>>> arr);
#endif