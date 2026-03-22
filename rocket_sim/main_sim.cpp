#include <iostream>
#include <cmath>
#include "objects.h"
#include "math.h"
#include <fstream>

int main (){

    constexpr vec1 rocket1_start_pos{
        .x = 0,
        .y = 0
    };
    constexpr vec1 rocket1_start_rot{
        .z = 0
    };

    double mass;
    double force;
    double start_ang;
    double bt;
    double dcoef;
    double area;
    double vel;
    double airden;
    double ang = start_ang + 0;      //place holders until
    double rocket_ang_deg_ps = 0;            // I add calcluations for them    

    double fx = calc_x(force, ang);
    double fy = calc_y(force, ang);

    double ax = calc_ax(fx, mass);
    double ay = calc_ay(fy, mass);

    double accel = calc_accel(force, mass);

    double t = 0.0;
    double dt = 0.1;
    double dur = 600;

    double vx = 0.0, vy = 0.0;
    double px = 0.0, py = 0.0;
    
    vel = sqrt((vx * vx) + (vy * vy));


    rocket rocket1;
    rocket1.init_rocket(rocket1_start_pos.x, rocket1_start_pos.y, rocket1_start_rot.z);
    rocket rocket2;
    
    while(t <= dur){
        t += dt;

        double cur_ax = 0.0, cur_ay = -9.8;
        
        if(t < bt){
            cur_ax += calc_ax(fx, mass);
            cur_ay += calc_ay(fy, mass);
        }

        vel = sqrt(vx * vx + vy * vy);

        if(vel > 0){
                if(py >= 90000)
                        airden = 0;
                 else if(py < 84852)
                        airden = 0.00001;
                 else if(py < 70000) 
                        airden = 0.00008;
                  else if(py < 60000) 
                        airden = 0.0003;
                  else if(py < 50000) 
                        airden = 0.001;
                  else if(py < 40000) 
                        airden = 0.004;
                  else if(py < 30000) 
                        airden = 0.018;
                  else if(py < 20000) 
                        airden = 0.088;
                  else if(py < 15000) 
                        airden = 0.195;
                  else if(py < 11000) 
                        airden = 0.364;
                  else if(py < 10000) 
                        airden = 0.413;
                  else if(py < 5000) 
                        airden = 0.736;
                  else if(py < 2000) 
                        airden = 1.007;
                  else if (py <1000) 
                        airden = 1.225;

                double drag = calc_drag(vel, airden, dcoef, area);
                cur_ax -= (drag/mass)*(vx/vel);
                cur_ay -= (drag/mass)*(vy/vel);
            }
        
        double ideal_ang_deg = calc_ideal_ang_deg(vx,vy);
        double ang_err = calc_ang_err(ideal_ang_deg, ang);
        double ang_vel_deg_ps = calc_vel_deg_ps(ang_err); // speed of rotation adjusted by gain (0.5)
            
        if (fabs(ang_vel_deg_ps) > 90.0f){
                ang_vel_deg_ps = copysign(90.0f, ang_vel_deg_ps); // max rotation speed
        }

        ang += ang_vel_deg_ps * dt;

        vx += cur_ax * dt;
        vy += cur_ay * dt;
        px += vx * dt;
        py += vy * dt;

        rocket1.set_pos(px, py);
        vec1 pos = rocket1.get_pos();

        rocket1.set_ang(ang);
        vec1 rot = rocket1.get_rot();

        if(py <= 0 && t > dt) break;

    }

    return 0;
}