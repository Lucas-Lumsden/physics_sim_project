#pragma once
#include "math.h"
#include "objects.h"
#include <algorithm>

struct SimState {

    float mass, force, bt, dcoef, area;  
    double airden;
    float start_pitch = 90.0f;
    float start_yaw = 0.0f;
    double thrust_ang = 0.0f;
    double aero_ang = 0.0f;          //place holders until i can determine how to calc the forces
    double gyro_ang = 0.0f;
    double ang = calc_ang(thrust_ang, aero_ang, gyro_ang, start_pitch);         //currently useless
    
    double fx = calc_x(force, ang, start_yaw);
    double fy = calc_y(force, ang);
    double fz = calc_z(force, ang, start_yaw);

    double t = 0.0, dt = 0.01, dur = 600.0;
    double vx = 0.0, vy = 0.0, vz = 0.0;
    double vel;
    double px = 0.0, py = 0.0, pz = 0.0;
    double test_dt = 0.1;                          
    bool launched = false;

// initial quaternion orientation
    double q_w = 1.0, q_x = 0.0, q_y = 0.0, q_z = 0.0;
    
// ang vel rad/s
    double omega_x = 0.0, omega_y = 0.0, omega_z = 0.0;
    
// current euler ang for render
    double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;

    rocket rocket1;
    rocket rocket2;

    void init(double i_mass, double i_force, double i_start_pitch, double i_start_yaw, double i_bt, double i_dcoef, double i_area){
        
        mass = i_mass;
        force = i_force;
        start_pitch = i_start_pitch; 
        start_yaw = i_start_yaw;
        bt = i_bt;
        dcoef = i_dcoef;
        area = i_area;

        fx = calc_x(force, start_pitch, start_yaw);
        fy = calc_y(force, start_pitch);
        fz = calc_z(force, start_pitch, start_yaw);
        airden = 1.225;
        launched = false;
        
        rocket1.init_rocket(i_start_pitch, i_start_yaw);       // CHECK for start params var

    }

    void launch(){
        vx = 0.0; vy = 0.0; vz = 0.0;
        px = 0.0; py = 0.0; pz = 0.0;
        vel = 0.0;
        t = 0;
        
// orientation to launch angle
        q_w = 1.0; q_x = 0.0; q_y = 0.0; q_z = 0.0;
        omega_x = 0.0; omega_y = 0.0; omega_z = 0.0;
        
//quaternions
        fx = calc_x(force, start_pitch, start_yaw);
        fy = calc_y(force, start_pitch);
        fz = calc_z(force, start_pitch, start_yaw);

        double fx_n = fx/force; 
        double fy_n = fy/force;
        double fz_n = fz/force;

        double dot = fy_n;
        double cross_x = fz_n;
        double cross_y = fy_n;
        double cross_z = -fx_n;

        double s = sqrt((1.0 + dot) * 2.0);
        q_w = s * 0.5;
        q_x = cross_x / s;
        q_y = cross_y / s;
        q_z = cross_z / s;

        double q_mag = sqrt(q_w*q_w + q_x*q_x + q_y*q_y + q_z*q_z);
        q_w /= q_mag; q_x /= q_mag; q_y /= q_mag; q_z /= q_mag;

        launched = true;
    }

    void reset(){
        px=0; py=0; pz=0;
        vx=0; vy=0; vz=0;
        vel=0;
        ang = start_pitch; 
        t=0;
        
// reset orientatiom
        q_w = 1.0; q_x = 0.0; q_y = 0.0; q_z = 0.0;
        omega_x = 0.0; omega_y = 0.0; omega_z = 0.0;
        roll_deg = 0.0; pitch_deg = 0.0; yaw_deg = 0.0;
        
        launched = false;
    }

    double calcScale(){

        double test_fx = calc_x(force, start_pitch, start_yaw);
        double test_fy = calc_y(force, start_pitch);
        double test_fz = calc_z(force, start_pitch, start_yaw);

        double test_px=0, test_py=0, test_pz=0;
        double test_vx=0, test_vy=0, test_vz=0; 
        double test_t=0, test_dt = 0.1; 
        double test_vel=0; 
        double test_ax, test_ay, test_az;
        double max_x=0, max_y=0, max_z = 0;
        double test_den;

        while(test_t <= dur){
            test_t += test_dt;
            test_ax = 0.0; test_ay = -9.8; test_az = 0;
            
            if(test_t < bt){
                test_ax += calc_ax(test_fx, mass);
                test_ay += calc_ay(test_fy, mass) + 9.8;
                test_az += calc_az(test_fz, mass);

            }
            test_vel = sqrt(test_vx*test_vx + test_vy*test_vy + test_vz*test_vz);
       
            if(test_vel > 0){
                if(test_py >= 90000)
                            test_den = 0;
                    else if(test_py >= 84852)
                            test_den = 0.00001;
                    else if(test_py >= 70000) 
                            test_den = 0.00008;
                    else if(test_py >= 60000) 
                            test_den = 0.0003;
                    else if(test_py >= 50000) 
                            test_den = 0.001;
                    else if(test_py >= 40000) 
                            test_den = 0.004;
                    else if(test_py >= 30000) 
                            test_den = 0.018;
                    else if(test_py >= 20000) 
                            test_den = 0.088;
                    else if(test_py >= 15000) 
                            test_den = 0.195;
                    else if(test_py >= 11000) 
                            test_den = 0.364;
                    else if(test_py >= 10000) 
                            test_den = 0.413;
                    else if(test_py >= 5000) 
                            test_den = 0.736;
                    else if(test_py >= 2000) 
                            test_den = 1.007;
                    else if (test_py >=1000) 
                            test_den = 1.112;
                    else test_den = 1.225;

                double drag = calc_drag(test_vel, test_den, dcoef, area);
                test_ax -= (drag/mass)*(test_vx/test_vel);
                test_ay -= (drag/mass)*(test_vy/test_vel);
                test_az -= (drag/mass)*(test_vz/test_vel);

            }
            test_vx += test_ax*test_dt; 
            test_vy += test_ay*test_dt; 
            test_vz += test_az*test_dt;
            test_px += test_vx*test_dt; 
            test_py += test_vy*test_dt; 
            test_pz += test_vz*test_dt;

            if(test_px > max_x) max_x = test_px;
            if(test_py > max_y) max_y = test_py;
            if(test_pz > max_z) max_z = test_pz;
            if(test_py <= 0 && test_t > test_dt) break;
        }
        return std::max({max_x, max_y, max_z}) * 1.2;
    }

    void step(){

        if(!launched || (py <= 0 && t > dt)) return;

        t += dt;
        double cur_ax = 0.0, cur_ay = -9.8, cur_az = 0.0;
        
        if(t < bt){

            cur_ax += calc_ax(fx, mass);
            cur_ay += calc_ay(fy, mass);
            cur_az += calc_az(fz, mass);
        }

    vel = sqrt((vx * vx) + (vy * vy) + (vz * vz));
        
        double tor_x = 0.0, tor_y = 0.0, tor_z = 0.0;   // CHECK this may be more complex than this
        double drag = 0.0;
        
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

                drag = calc_drag(vel, airden, dcoef, area);
                    cur_ax -= (drag/mass)*(vx/vel);
                    cur_ay -= (drag/mass)*(vy/vel);
                    cur_az -= (drag/mass)*(vz/vel);

                if(vel > 1.0){
                        
// nose direction
    double cp_offset = 0.5;

    double drag_fx = -(drag) * (vx/vel);
    double drag_fy = -(drag) * (vy/vel);
    double drag_fz = -(drag) * (vz/vel);

    double nx = 2.0*(q_x*q_y + q_w*q_z);
    double ny = 2.0*(q_x*q_x + q_z*q_z);
    double nz = 1.0 - 2.0*(q_y*q_z - q_w*q_x);

    tor_x += (ny * cp_offset) * drag_fz - (nz * cp_offset) * drag_fy;
    tor_y += (nz * cp_offset) * drag_fx - (nx * cp_offset) * drag_fz;
    tor_z += (nx * cp_offset) * drag_fy - (ny * cp_offset) * drag_fx;

// angle between nose and velocity
        double nose_vel_dot = nx*(vx/vel) + ny*(vy/vel) + nz*(vz/vel);
        double angle_error = acos(glm::clamp(nose_vel_dot, -1.0, 1.0));

// damping scales with misalignment
        double damp = 2.0 + angle_error * 5.0;
        omega_x *= (1.0 - damp * dt);
        omega_y *= (1.0 - damp * dt);
        omega_z *= (1.0 - damp * dt);

        }       
}

        vx += cur_ax * dt;
        vy += cur_ay * dt;
        vz += cur_az * dt;
        px += vx * dt;
        py += vy * dt;
        pz += vz * dt;
        
// ang accel to ang vel
double moi = 10.0;
        omega_x += (tor_x / moi) * dt;
        omega_y += (tor_y / moi) * dt;
        omega_z += (tor_z / moi) * dt;
        
// quaternion from ang vel
        double dq_w = 0.5 * (-q_x * omega_x - q_y * omega_y - q_z * omega_z);
        double dq_x = 0.5 * (q_w * omega_x + q_y * omega_z - q_z * omega_y);
        double dq_y = 0.5 * (q_w * omega_y - q_x * omega_z + q_z * omega_x);
        double dq_z = 0.5 * (q_w * omega_z + q_x * omega_y - q_y * omega_x);
        
        q_w += dq_w * dt;
        q_x += dq_x * dt;
        q_y += dq_y * dt;
        q_z += dq_z * dt;
        
// normalize quaternion
        double q_mag = sqrt(q_w * q_w + q_x * q_x + q_y * q_y + q_z * q_z);
        if(q_mag > 0.0){
            q_w /= q_mag;
            q_x /= q_mag;
            q_y /= q_mag;
            q_z /= q_mag;
        }
        
        // convert Euler angle
        EulerAngles angles = ToEulerAngles(q_w, q_x, q_y, q_z);
        roll_deg = angles.roll * 57.2958;     // rad to deg
        pitch_deg = angles.pitch * 57.2958;
        yaw_deg = angles.yaw * 57.2958;

        rocket1.set_pos(px, py, pz);
        rocket1.set_ang(roll_deg, pitch_deg, yaw_deg);

        // console debug
        
        static int debug_count = 0;
        if(debug_count++ % 10 == 0){
                std::cout << "t=" << t << " py=" << py << " vy=" << vy << " roll=" << roll_deg 
                      << " pitch=" << pitch_deg << " yaw=" << yaw_deg << std::endl;
   
                std::cout << "t=" << t << " py=" << py << " vy=" << vy << " roll=" << roll_deg 
                        << " pitch=" << pitch_deg << " yaw=" << yaw_deg << std::endl;
        }
    }
};