#include <iostream>
#include <cmath>
#include "objects.h"
#include "math.h"

double calc_x(double force, double angle){

    double rad = angle * 0.0174533;
    return force * std::cos(rad);

};

double calc_y(double force, double angle){
    
    double rad = angle * 0.0174533;
    return force * std::sin(rad);

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

double calc_drag(double vel, double airden, double dcoef, double area){

    return 0.5 * airden * (vel * vel) * dcoef * area;

}

double calc_ang(double thrust_ang, double aero_ang, double gyro_ang, double start_ang){

    return thrust_ang * aero_ang * gyro_ang + start_ang; //placeholder

}

double calc_ideal_ang_deg(double vx, double vy){

    return atan2(vx, vy) * 180.f / 3.14159265358979;

}

double calc_ang_err(double ideal_ang_deg, double rocket_ang_deg){

    return ideal_ang_deg - rocket_ang_deg;

}

double calc_vel_deg_ps(double ang_err){

    return ang_err * 0.5f;

}