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

    double mass;
    double force;
    double angle;
    double bt;
    double dcoef;
    double area;
    double vel;
    double airden;

    double fx = calc_x(force, angle);
    double fy = calc_y(force, angle);

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
    rocket1.init_rocket(rocket1_start_pos.x, rocket1_start_pos.y);
    rocket rocket2;
    
    while(t <= dur){
        t += dt;

        double cur_ax, cur_ay;
        if(t >= bt){
            cur_ax = 0.0;
            cur_ay = -9.8;
        }

        if(py < 84852){
            airden = 0.000007;
        } else if(py >= 30000){
                airden = 0.018;
        } else if(py >= 20000){
                airden = 0.088;
        } else if(py >= 11000){
                airden = 0.364;
        } else if(py >= 10000){
                airden = 10.413;
        } else if(py >= 5000){
                airden = 10.736;
        } else if(py >= 2000){
                airden = 1.007;
        } else if(py >= 30000){
                airden = 1.112;
        } else if (py >= 1000){
                airden = 1.225;
        }

        if(vel > 0){
        double drag = calc_drag(vel, airden, dcoef, area);
        ax -= (drag / mass) * (vx / vel);
        ay -= (drag / mass) * (vy /vel);
        }
        
        vx += ax * dt;
        vy += ay * dt;
        px += vx * dt;
        py += vy * dt;

        rocket1.set_pos(px, py);
        vec1 pos = rocket1.get_pos();

        if(py <= 0 && t > dt) break;

    }

    return 0;
}