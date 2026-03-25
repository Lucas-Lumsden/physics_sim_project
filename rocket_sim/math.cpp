#include <iostream>
#include <cmath>
#include <numeric>
#include <algorithm>
#include "objects.h"
#include "math.h"

double calc_x(double force, double angle, double start_yaw){

    double rad_angle = angle * 0.0174533;
    double rad_start_yaw = start_yaw * 0.0174533;
    double horizontal = force * std::cos(rad_angle);
    return horizontal * std::cos(rad_start_yaw);

};

double calc_y(double force, double angle){
    
    double rad = angle * 0.0174533;
    return force * std::sin(rad);

};

double calc_z(double force, double angle, double start_yaw){
    
    double rad_angle = angle * 0.0174533;
    double rad_start_yaw = start_yaw * 0.0174533;
    double horizontal = force * std::cos(rad_angle);
    return horizontal * std::sin(rad_start_yaw);

};

double calc_vel(double vi, double accel, double dt){

    return vi + accel * dt;
};

double calc_pos(double xi, double vi, double dt){

    return xi + vi * dt;
};

double calc_accel(double force, double mass){
    double accel;
    return accel = force / mass;

};

double calc_ax(double fx, double mass){

    return fx / mass;

};

double calc_ay(double fy, double mass){

    return (fy / mass);

}

double calc_az(double fz, double mass){

    return (fz / mass);

}

double calc_drag(double vel, double airden, double dcoef, double area){

    return 0.5 * airden * (vel * vel) * dcoef * area;

}

double calc_ang(double thrust_ang, double aero_ang, double gyro_ang, double pitch_deg){

    return thrust_ang * aero_ang * gyro_ang + pitch_deg; //placeholder

}

EulerAngles ToEulerAngles(double w, double x, double y, double z){
    EulerAngles angles;

    //roll
    double sinr_cosp = 2 * (w*x+y*z);
    double cosr_cosp = 1 - 2 * (x*x + y*y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    //pitch
    double sinp = 2 * (w * y - z * x);
    if(abs(sinp) >= 1)
        angles.pitch = copysign(3.14159265358979 / 2, sinp);
    else angles.pitch = asin(sinp);

    //yaw
    double siny_cosp = 2 * (w*z+x*y);
    double cosy_cosp = 1 - 2 * (y*y + z*z);
    angles.yaw = atan2(siny_cosp, cosy_cosp);

    return angles;
}