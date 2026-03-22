#pragma once
double calc_x(double force, double angle);
double calc_y(double force, double angle);
double calc_accel(double mass, double force);
double calc_vel(double vi, double accel, double dt);
double calc_pos(double xi, double vi, double dt);
double calc_ax(double fx, double mass);
double calc_ay(double fy, double mass);
double calc_drag(double vel, double airden, double dcoef, double area);

double calc_ang(double thrust_ang, double aero_ang, double gyro_ang, double start_ang);
double calc_rot_vel(double thrust_ang, double aero_ang, double gyro_ang);
double calc_ideal_ang_deg(double vx, double vy);

double calc_ang_err(double ideal_ang_deg, double rocket_ang_deg);
double calc_vel_deg_ps(double ang_err);