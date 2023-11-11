#include "corner_points.h"

bool sort_surface_yz(const pair<double,pair<double,double>> &a, pair<double,pair<double,double>> &b){
    return a.first < b.first;
}

bool sort_surface_xz(const pair<double,pair<double,double>> &a, pair<double,pair<double,double>> &b){
    return a.second.first < b.second.first;
}

bool sort_surface_xy(const pair<double,pair<double,double>> &a, pair<double,pair<double,double>> &b){
    return a.second.second < b.second.second;
}

void surface_display(vector<pair<double,pair<double,double>>>& arr){
    for(auto& x : arr)
        cout << x.first << " " << x.second.first << " " << x.second.second << endl;
    cout << endl;
}
vector<pair<double,pair<double,double>>> get_surface1(vector<pair<double,pair<double,double>>> arr){
    sort(arr.begin(), arr.end(), sort_surface_yz);
    vector<pair<double,pair<double,double>>> surface_1;
    surface_1.assign(arr.begin(), arr.begin()+4);
    return surface_1;
 }

vector<pair<double,pair<double,double>>> get_surface2(vector<pair<double,pair<double,double>>> arr){
    sort(arr.begin(), arr.end(), sort_surface_yz);
    vector<pair<double,pair<double,double>>> surface_2;
    surface_2.assign(arr.begin()+4, arr.end());
    return surface_2;
}

vector<pair<double,pair<double,double>>> get_surface3(vector<pair<double,pair<double,double>>> arr){
    sort(arr.begin(), arr.end(), sort_surface_xz);
    vector<pair<double,pair<double,double>>> surface_3;
    surface_3.assign(arr.begin(), arr.begin()+4);
    return surface_3;
}

vector<pair<double,pair<double,double>>> get_surface4(vector<pair<double,pair<double,double>>> arr){
    sort(arr.begin(), arr.end(), sort_surface_xz);
    vector<pair<double,pair<double,double>>> surface_4;
    surface_4.assign(arr.begin()+4, arr.end());
    return surface_4;
}

vector<pair<double,pair<double,double>>> get_surface5(vector<pair<double,pair<double,double>>> arr){
    sort(arr.begin(), arr.end(), sort_surface_xy);
    vector<pair<double,pair<double,double>>> surface_5;
    surface_5.assign(arr.begin()+4, arr.end());
    return surface_5;
}
