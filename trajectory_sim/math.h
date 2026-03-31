#ifndef math_h
#define matgh_h
#include "objects.h"

double calc_x(double force, double ang, double start_yaw);
double calc_y(double force, double ang);
double calc_z(double force, double ang, double start_yaw);

double calc_ax(double fx, double mass);
double calc_ay(double fy, double mass);
double calc_az(double fz, double mass);

double calc_drag(double vel, double airden, double dcoef, double area);

double calc_ang(double pitch_deg);

EulerAngles ToEulerAngles(double w, double x, double y, double z);

#endif