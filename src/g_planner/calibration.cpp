#include "calibration.h"

float diagonal(int len, int width){
    return sqrt(pow(len,2) + pow(width,2));
}

// inputs to main file
double dist_to_drone(int PPI, int pixel_len, int pixel_width, float FOV){
    double height = (2 * diagonal(pixel_len,pixel_width))/(5*PPI*tan(FOV/2));
    // inch to meter conversion
    height *= 0.0254;
    return height;
}

void area_captured(int PPI, int pixel_len, int pixel_width, float FOV, float dim[]){
    FOV *= M_PI/180;
    double height = dist_to_drone(PPI,pixel_len,pixel_width,FOV);
    // dim[0] is width and dim[1] is length
    dim[0] = 2 * height * tan(FOV/2);
    dim[1] = (3 * dim[0])/4;
}