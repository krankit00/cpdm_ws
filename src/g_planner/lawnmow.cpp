#include "lawnmow.h"

bool asc_sort_points_yz(const pair<double,pair<double,double>>& a, const pair<double,pair<double,double>>& b){
    if (a.second.first < b.second.first){
        return true;
    }
    if (a.second.first == b.second.first){
        return a.second.second < b.second.second;
    }
    return false;
}

bool desc_sort_points_yz(const pair<double,pair<double,double>>& a, const pair<double,pair<double,double>>& b){
    if (a.second.first > b.second.first){
        return true;
    }
    if (a.second.first == b.second.first){
        return a.second.second > b.second.second;
    }
    return false;
}

bool asc_sort_points_xz(const pair<double,pair<double,double>>& a, const pair<double,pair<double,double>>& b){
    if (a.first < b.first){
        return true;
    }
    if (a.first == b.first){
        return a.second.second < b.second.second;
    }
    return false;
}
bool desc_sort_points_xz(const pair<double,pair<double,double>>& a, const pair<double,pair<double,double>>& b){
    if (a.first > b.first){
        return true;
    }
    if (a.first == b.first){
        return a.second.second > b.second.second;
    }
    return false;
}

bool asc_sort_points_xy(const pair<double,pair<double,double>>& a, const pair<double,pair<double,double>>& b){
    if (a.first < b.first){
        return true;
    }
    if (a.first == b.first){
        return a.second.first < b.second.first;
    }
    return false;
}

bool desc_sort_points_xy(const pair<double,pair<double,double>>& a, const pair<double,pair<double,double>>& b){
    if (a.first > b.first){
        return true;
    }
    if (a.first == b.first){
        return a.second.first > b.second.first;
    }
    return false;
}

void display_sur(vector<pair<double,pair<double,double>>>& arr){
    for(auto& x : arr)
        cout << x.first << " " << x.second.first << " " << x.second.second << endl;
    cout << endl;
}

vector<vector<double> > find_waypoint(float width_cap, float len_cap, vector<pair<double,pair<double,double>>> surface,float h_dist,
                                                float v_dist, bool ascending, float drone_dist, bool& mirror, float& path_length){
    
    vector<vector<double> > diagonal_points;
    vector<vector<double>> point;
    char const_cord;
    for (int i = 0; i < 4; i++){
        for (int j = 1; j < 4; j++){
            if (surface[i].first == surface[j].first and surface[i].second.first != surface[j].second.first
                and surface[i].second.second != surface[j].second.second){
                    cout<<"Surface is in YZ Plane\n";
                    const_cord = 'x';
                    if (ascending) sort(surface.begin(),surface.end(),asc_sort_points_yz);
                    else sort(surface.begin(),surface.end(), desc_sort_points_yz);
                    diagonal_points.push_back({surface[0].second.first,surface[0].second.second});
                    diagonal_points.push_back({surface[3].second.first,surface[3].second.second});
                    break;

            }
            if (surface[i].first != surface[j].first and surface[i].second.first == surface[j].second.first
                and surface[i].second.second != surface[j].second.second){
                    cout<<"Surface is in XZ Plane\n";
                    const_cord = 'y';
                    if (ascending) sort(surface.begin(),surface.end(),asc_sort_points_xz);
                    else sort(surface.begin(),surface.end(),desc_sort_points_xz);
                    diagonal_points.push_back({surface[0].first,surface[0].second.second});
                    diagonal_points.push_back({surface[3].first,surface[3].second.second});
                    break;
            }
            if (surface[i].first != surface[j].first and surface[i].second.first != surface[j].second.first
                and surface[i].second.second == surface[j].second.second){
                    cout<<"Surface is in XY Plane\n";
                    const_cord = 'z';
                    if (ascending) sort(surface.begin(),surface.end(),asc_sort_points_xy);
                    else sort(surface.begin(),surface.end(),desc_sort_points_xy);
                    diagonal_points.push_back({surface[0].first,surface[0].second.first});
                    diagonal_points.push_back({surface[3].first,surface[3].second.first});
                    break;
            }
        }
        break;
    }
    path_length = len_cap/2;
    // display_sur(surface);
    vector<double> tmp;
    int flag = 1;
    // for (auto& x: diagonal_points){
    //     for (auto& y: x){
    //         cout<<y<<" ";
    //     }
    //     cout<<"\n";
    // }
    if (const_cord == 'z' and ascending and !mirror){
        float curr_dY = surface[0].second.first + len_cap/2;
        float curr_dX = surface[0].first + width_cap/2;
        float curr_dZ = surface[0].second.second + drone_dist;
        for (;curr_dX < diagonal_points[1][0]; curr_dX += width_cap - v_dist){

            if (flag){
                for (;curr_dY < diagonal_points[1][1]; curr_dY += len_cap - h_dist){
                    tmp.push_back(curr_dX);
                    tmp.push_back(curr_dY);
                    tmp.push_back(curr_dZ);
                    point.push_back(tmp);
                    tmp.clear();
                    path_length += len_cap - h_dist;
                }
                flag = 0;
            }
            else{
                for (;curr_dY > diagonal_points[0][1]; curr_dY -= len_cap - h_dist){
                    tmp.push_back(curr_dX);
                    tmp.push_back(curr_dY);
                    tmp.push_back(curr_dZ);
                    point.push_back(tmp);
                    tmp.clear();
                    path_length += len_cap - h_dist;
                }
                flag = 1;
            }
            path_length += width_cap - v_dist;
        }
        mirror = true;

    }
    // if (const_cord == 'z' and !ascending){
    //     float curr_dY = surface[0].second.first + width_cap/2;
    //     float curr_dX = surface[0].first + len_cap/2;
    //     float curr_dZ = surface[0].second.second + drone_dist;
    //     for (;curr_dY > diagonal_points[1][1]; curr_dY -= width_cap - v_dist){

    //         if (flag){
    //             for (;curr_dX > diagonal_points[1][0]; curr_dX -= len_cap - h_dist){
    //                 tmp.push_back(curr_dX);
    //                 tmp.push_back(curr_dY);
    //                 tmp.push_back(curr_dZ);
    //                 point.push_back(tmp);
    //                 tmp.clear();
    //             }
    //             flag = 0;
    //         }
    //         else{
    //             for (;curr_dX < diagonal_points[0][0]; curr_dX += len_cap - h_dist){
    //                 tmp.push_back(curr_dX);
    //                 tmp.push_back(curr_dY);
    //                 tmp.push_back(curr_dZ);
    //                 point.push_back(tmp);
    //                 tmp.clear();
    //             }
    //             flag = 1;
    //         }
    //     }

    // }
    else if (const_cord == 'x' and !ascending and !mirror){
        float curr_dZ = surface[0].second.second + width_cap/2;
        float curr_dY = surface[0].second.first + len_cap/2;
        float curr_dX = surface[0].first + drone_dist;
        for (;curr_dZ > diagonal_points[1][1]; curr_dZ -= width_cap - v_dist){

            if (flag){
                for (;curr_dY > diagonal_points[1][0]; curr_dY -= len_cap - h_dist){
                    tmp.push_back(curr_dX);
                    tmp.push_back(curr_dY);
                    tmp.push_back(curr_dZ);
                    point.push_back(tmp);
                    tmp.clear();
                    path_length += len_cap - h_dist;
                }
                flag = 0;
            }
            else{
                for (;curr_dY < diagonal_points[0][0]; curr_dY += len_cap - h_dist){
                    tmp.push_back(curr_dX);
                    tmp.push_back(curr_dY);
                    tmp.push_back(curr_dZ);
                    point.push_back(tmp);
                    tmp.clear();
                    path_length += len_cap - h_dist;
                }
                flag = 1;
            }
            path_length += width_cap - v_dist;
        }
        mirror = true;
    }
    else if (const_cord == 'x' and !ascending and mirror){
        float curr_dZ = surface[0].second.second + width_cap/2;
        float curr_dY = surface[3].second.first + len_cap/2;
        float curr_dX = surface[0].first + drone_dist;
        for (;curr_dZ > diagonal_points[1][1]; curr_dZ -= width_cap - v_dist){

            if (flag){
                for (;curr_dY < diagonal_points[0][0]; curr_dY += len_cap - h_dist){
                    tmp.push_back(curr_dX);
                    tmp.push_back(curr_dY);
                    tmp.push_back(curr_dZ);
                    point.push_back(tmp);
                    tmp.clear();
                    path_length += len_cap - h_dist;
                }
                flag = 0;
            }
            else{
                for (;curr_dY > diagonal_points[1][0]; curr_dY -= len_cap - h_dist){
                    tmp.push_back(curr_dX);
                    tmp.push_back(curr_dY);
                    tmp.push_back(curr_dZ);
                    point.push_back(tmp);
                    tmp.clear();
                    path_length += len_cap - h_dist;
                }
                flag = 1;
            }
            path_length += width_cap - v_dist;
        }
        mirror = false;
    }
    else if (const_cord == 'y' and ascending and !mirror){
        float curr_dZ = surface[0].second.second + width_cap/2;
        float curr_dX = surface[0].first + len_cap/2;
        float curr_dY = surface[0].second.first + drone_dist;
        for (;curr_dZ < diagonal_points[1][1]; curr_dZ += width_cap - v_dist){

            if (flag){
                for (;curr_dX < diagonal_points[1][0]; curr_dX += len_cap - h_dist){
                    tmp.push_back(curr_dX);
                    tmp.push_back(curr_dY);
                    tmp.push_back(curr_dZ);
                    point.push_back(tmp);
                    tmp.clear();
                    path_length += len_cap - h_dist;
                }
                flag = 0;
            }
            else{
                for (;curr_dX > diagonal_points[0][0]; curr_dX -= len_cap - h_dist){
                    tmp.push_back(curr_dX);
                    tmp.push_back(curr_dY);
                    tmp.push_back(curr_dZ);
                    point.push_back(tmp);
                    tmp.clear();
                    path_length += len_cap - h_dist;
                }
                flag = 1;
            }
            path_length += width_cap - v_dist;
        }
        mirror = true;
    }
    else if (const_cord == 'y' and ascending and mirror){
        float curr_dZ = surface[0].second.second + width_cap/2;
        float curr_dX = surface[3].first + len_cap/2;
        float curr_dY = surface[0].second.first + drone_dist;
        for (;curr_dZ < diagonal_points[1][1]; curr_dZ += width_cap - v_dist){

            if (flag){
                for (;curr_dX > diagonal_points[0][0]; curr_dX -= len_cap - h_dist){
                    tmp.push_back(curr_dX);
                    tmp.push_back(curr_dY);
                    tmp.push_back(curr_dZ);
                    point.push_back(tmp);
                    tmp.clear();
                    path_length += len_cap - h_dist;
                }
                flag = 0;
            }
            else{
                for (;curr_dX < diagonal_points[1][0]; curr_dX += len_cap - h_dist){
                    tmp.push_back(curr_dX);
                    tmp.push_back(curr_dY);
                    tmp.push_back(curr_dZ);
                    point.push_back(tmp);
                    tmp.clear();
                    path_length += len_cap - h_dist;
                }
                flag = 1;
            }
            path_length += width_cap - v_dist;
        }
        mirror = false;
    }
    return point;
}
