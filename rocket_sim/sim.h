#pragma once
#include "math.h"
#include "objects.h"

struct SimState {

    float mass, force, bt, dcoef, area;  
    double airden;
    float start_ang;
    double thrust_ang = 0.0f;
    double aero_ang = 0.0f;          //place holders until i can determine how to calc the forces
    double gyro_ang = 0.0f;
    double ang = calc_ang(thrust_ang, aero_ang, gyro_ang, start_ang);
    double rocket_ang_deg_ps = 10;    
    
    double fx = calc_x(force, ang);
    double fy = calc_y(force, ang);
    double t = 0.0, dt = 0.01, dur = 600.0;
    double vx = 0.0, vy = 0.0, vel = 0.0;
    double px = 0.0, py = 0.0;
    double test_dt = 0.1;                          
    bool launched = false;

    rocket rocket1;
    rocket rocket2;

    void init(double i_mass, double i_force, double i_start_ang, double i_bt, double i_dcoef, double i_area){
        
        mass = i_mass;
        force = i_force;
        start_ang = i_start_ang; 
        bt = i_bt;
        dcoef = i_dcoef;
        area = i_area;

        fx = calc_x(force, ang);
        fy = calc_y(force, ang);
        airden = 1.225;
        launched = false;
        rocket1.init_rocket(0.0f, 0.0f, start_ang);

    }

    void launch(){
        vx = 0.0, vy = 0.0,
        px = 0.0, py = 0.0;
        vel = 0.0;
        ang = start_ang;
        t = 0;
        fx = calc_x(force, ang);
        fy = calc_y(force, ang);
        launched = true;
    }

    void reset(){
        px=0; py=0; 
        vx=0; vy=0; 
        vel=0;
        ang = start_ang; 
        t=0;
        launched = false;
    }

    double calcScale(){

        double test_fx = calc_x(force, start_ang);
        double test_fy = calc_y(force, start_ang);

        double test_px=0, test_py=0; 
        double test_vx=0, test_vy=0;
        double test_t=0, test_dt = 0.1; 
        double test_vel=0; 
        double test_ax, test_ay;
        double max_x=0, max_y=0;
        double test_den;

        while(test_t <= dur){
            test_t += test_dt;
            test_ax = 0.0; test_ay = -9.8;
            if(test_t < bt){
                test_ax += calc_ax(test_fx, mass);
                test_ay += calc_ay(test_fy, mass) + 9.8;
            }
            test_vel = sqrt(test_vx*test_vx + test_vy*test_vy);
            if(test_vel > 0){
                if(test_py >= 100000){
                        test_den = 0;
                }   else if(test_py >= 84852){
                        test_den = 0.000007;
                }   else if(test_py >= 30000){
                        test_den = 0.018;
                }   else if(test_py >= 20000){
                        test_den = 0.088;
                }   else if(test_py >= 11000){
                        test_den = 0.364;
                }   else if(test_py >= 10000){
                      test_den = 10.413;
                }   else if(test_py >= 5000){
                        test_den = 10.736;
                }   else if(test_py >= 2000){
                        test_den = 1.007;
                }   else if(test_py >= 30000){
                        test_den = 1.112;
                }   else if (test_py >= 1000){
                        test_den = 1.225;
                }

                double drag = calc_drag(test_vel, test_den, dcoef, area);
                test_ax -= (drag/mass)*(test_vx/test_vel);
                test_ay -= (drag/mass)*(test_vy/test_vel);
            }
            test_vx += test_ax*test_dt; test_vy += test_ay*test_dt;
            test_px += test_vx*test_dt; test_py += test_vy*test_dt;
            if(test_px > max_x) max_x = test_px;
            if(test_py > max_y) max_y = test_py;
            if(test_py < 0 && test_t > test_dt) break;
        }
        return std::max(max_x, max_y) * 1.2;
    }

    void step(){
        if(!launched || (py < 0 && t > dt)) return;

        t += dt;
        double cur_ax = 0.0, cur_ay = -9.8;
        if(t < bt){
            cur_ax += calc_ax(fx, mass);
            cur_ay += calc_ay(fy, mass);
        }
    vel = sqrt((vx * vx) + (vy * vy));

        if(vel > 0){
            if(py >= 90000)
                            airden = 0;
                    else if(py >= 84852)
                            airden = 0.00001;
                    else if(py >= 70000) 
                            airden = 0.00008;
                    else if(py >= 60000) 
                            airden = 0.0003;
                    else if(py >= 50000) 
                            airden = 0.001;
                    else if(py >= 40000) 
                            airden = 0.004;
                    else if(py >= 30000) 
                            airden = 0.018;
                    else if(py >= 20000) 
                            airden = 0.088;
                    else if(py >= 15000) 
                            airden = 0.195;
                    else if(py >= 11000) 
                            airden = 0.364;
                    else if(py >= 10000) 
                            airden = 0.413;
                    else if(py >= 5000) 
                            airden = 0.736;
                    else if(py >= 2000) 
                            airden = 1.007;
                    else if (py >=1000) 
                            airden = 1.112;
                    else airden = 1.225;

                    double drag = calc_drag(vel, airden, dcoef, area);
                    cur_ax -= (drag/mass)*(vx/vel);
                    cur_ay -= (drag/mass)*(vy/vel);
        }
        
        double ideal_ang_deg = calc_ideal_ang_deg(vx,vy);
        double ang_err = calc_ang_err(ideal_ang_deg, ang);
        double ang_vel_deg_ps = calc_vel_deg_ps(ang_err); // speed of rotation adjusted by gain (0.5)
            
        if (fabs(ang_vel_deg_ps) > 90.0f)
                ang_vel_deg_ps = copysign(90.0f, ang_vel_deg_ps); // max rotation speed
        
        ang += ang_vel_deg_ps * dt;

        vx += cur_ax * dt;
        vy += cur_ay * dt;
        px += vx * dt;
        py += vy * dt;

        rocket1.set_pos(px, py);
        rocket1.set_ang(ang);

    }
};