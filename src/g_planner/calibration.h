#include <iostream>
#include <math.h>

#ifndef CALIBRATION_H
#define CALIBRATION_H
float diagonal(int len, int width);
double dist_to_drone(int PPI, int pixel_len, int pixel_width, float FOV);
void area_captured(int PPI, int pixel_len, int pixel_width, float FOV, float dim[]);
#endif