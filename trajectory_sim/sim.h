#pragma once
#include "math.h"
#include "objects.h"
#include <algorithm>
#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

struct SimState {

    float mass, force, bt, dcoef, area;  
    double airden;
    float start_pitch = 90.0f;
    float start_yaw = 0.0f;
    double ang = calc_ang(start_pitch); //currently only sets starting ang
    
    double fx = calc_x(force, ang, start_yaw);
    double fy = calc_y(force, ang);
    double fz = calc_z(force, ang, start_yaw);

    //aerodynamic normal force coef slope (very rough) larger = stronger alignment with airflow
    //used to generate stabilizing torque (see step)
    double cn_alpha = 0.8;
    
    //current aero stability uses cross(b, vhat) torque instead
    double cg_to_cp_m = -0.1;
    
    //moment of inertia (kg*m^2). placeholder until compute geometry
    double moi = 10.0;

    //angular damping coefficient (N*m*s). Acts like "rotational drag" to keep it from going unstable
    double ang_damp = 10.0;
    
    //constant wind in world frame (m/s)
    double wind_x = 0.0, wind_y = 0.0, wind_z = 0.0;

    //launch direction unit vector (world). used for debug and optional constraints (currently none)
    double launch_dir_x = 0.0, launch_dir_y = 1.0, launch_dir_z = 0.0;

    double t = 0.0, dt = 0.01, dur = 600.0;
    double vx = 0.0, vy = 0.0, vz = 0.0;
    double vel;
    double px = 0.0, py = 0.0, pz = 0.0;
    double test_dt = 0.1;                          
    bool launched = false;

    //initial quaternion orientation
    double q_w = 1.0, q_x = 0.0, q_y = 0.0, q_z = 0.0;
    
    //ang vel rad/s
    double omega_x = 0.0, omega_y = 0.0, omega_z = 0.0;
    
    // current euler ang for render
    double roll_deg = 0.0, pitch_deg = 0.0, yaw_deg = 0.0;

    //object2 physics state
    double px2 = 0.0, py2 = 0.0, pz2 = 0.0;
    double vx2 = 0.0, vy2 = 0.0, vz2 = 0.0;
    double vel2 = 0.0;

    //object2 orientation
    double q_w2 = 1.0, q_x2 = 0.0, q_y2 = 0.0, q_z2 = 0.0;
    double omega_x2 = 0.0, omega_y2 = 0.0, omega_z2 = 0.0;
    double roll_deg2 = 0.0, pitch_deg2 = 0.0, yaw_deg2 = 0.0;

    //object2 launch direction
    double launch_dir_x2 = 0.0, launch_dir_y2 = 1.0, launch_dir_z2 = 0.0;

    //turn rate, higher = faster turning
    double pursuit_turn_rate = 1.0;

    //delay before object2 starts pursuing
    double pursuit_delay = 2.0;

    //object2 starting position offset from object1 (meters)
    double obj2_start_x = 1000.0;
    double obj2_start_y = 500.0;
    double obj2_start_z = 1000.0;

    object object1;
    object object2;

    static double clamp01(double v, double lo, double hi){
        return (v < lo) ? lo : ((v > hi) ? hi : v);
    }

    //rotate vector v by quaternion q (world = q * v * q^-1), assumes q is normalized
    static void quatRotate(
        double qw, 
        double qx, 
        double qy, 
        double qz,
        double vx, 
        double vy, 
        double vz,
        double &ox, 
        double &oy, 
        double &oz)

        {

        const double tx = 2.0 * (qy * vz - qz * vy);
        const double ty = 2.0 * (qz * vx - qx * vz);
        const double tz = 2.0 * (qx * vy - qy * vx);

        ox = vx + qw * tx + (qy * tz - qz * ty);
        oy = vy + qw * ty + (qz * tx - qx * tz);
        oz = vz + qw * tz + (qx * ty - qy * tx);

    }

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
        
        object1.init_object(i_start_pitch, i_start_yaw);
        object2.init_object(i_start_pitch, i_start_yaw);

    }

    void launch(){
        vx = 0.0; vy = 0.0; vz = 0.0;
        px = 0.0; py = 0.0; pz = 0.0;
        vel = 0.0;
        t = 0;

        //orientation to launch angle
        //body +Y axis (nose) points along pitch+yaw direction
        q_w = 1.0; q_x = 0.0; q_y = 0.0; q_z = 0.0;
        omega_x = 0.0; omega_y = 0.0; omega_z = 0.0;

        //compute desired world direction from pitch/yaw
        //pitch: 90deg = straight up (+Y), 0deg = horizontal
        //yaw: rotation around +Y
        const double pitch_rad = start_pitch * 0.017453292519943295;
        const double yaw_rad = start_yaw * 0.017453292519943295;

        double dir_x = cos(pitch_rad) * cos(yaw_rad);
        double dir_y = sin(pitch_rad);
        double dir_z = cos(pitch_rad) * sin(yaw_rad);

        const double dmag = sqrt(dir_x * dir_x + dir_y * dir_y + dir_z * dir_z);
        if(dmag > 0.0){ dir_x /= dmag; dir_y /= dmag; dir_z /= dmag; }

        launch_dir_x = dir_x; launch_dir_y = dir_y; launch_dir_z = dir_z;

        //quaternion rotates (0,1,0) to (dir)
        //prevents large initial AoA
        const double from_x = 0.0, from_y = 1.0, from_z = 0.0;
        const double dot = clamp01(from_x * dir_x + from_y * dir_y + from_z * dir_z, -1.0, 1.0);

        double cross_x = from_y * dir_z - from_z * dir_y;
        double cross_y = from_z * dir_x - from_x * dir_z;
        double cross_z = from_x * dir_y - from_y * dir_x;

        if(dot < -0.999999){

            //180-degree flip: arbitrary axis perpendicular to (use +X)
            q_w = 0.0; q_x = 1.0; q_y = 0.0; q_z = 0.0;

        } else {

            q_w = 1.0 + dot;
            q_x = cross_x;
            q_y = cross_y;
            q_z = cross_z;
            const double q_mag = sqrt(q_w * q_w + q_x * q_x + q_y * q_y + q_z * q_z);
            
            if(q_mag > 0.0){ q_w /= q_mag; q_x /= q_mag; 
                q_y /= q_mag; q_z /= q_mag; 
            }

        }

        //launch object2 with same start conditions
        //object2 is offset to the side/forward to start pursuit
        vx2 = 0.0; vy2 = 0.0; vz2 = 0.0;
        px2 = obj2_start_x; py2 = obj2_start_y; pz2 = obj2_start_z;
        vel2 = 0.0;
        q_w2 = q_w; q_x2 = q_x; q_y2 = q_y; q_z2 = q_z;
        omega_x2 = 0.0; omega_y2 = 0.0; omega_z2 = 0.0;
        launch_dir_x2 = dir_x; launch_dir_y2 = dir_y; launch_dir_z2 = dir_z;

        object2.set_pos(px2, py2, pz2);
        object2.set_ang(roll_deg2, pitch_deg2, yaw_deg2);

        launched = true;

    }

    void reset(){
        px = 0; py = 0; pz = 0;
        vx = 0; vy = 0; vz = 0;
        vel = 0;
        ang = start_pitch;
        t = 0;

        //reset orientatiom
        q_w = 1.0; q_x = 0.0; q_y = 0.0; q_z = 0.0;
        omega_x = 0.0; omega_y = 0.0; omega_z = 0.0;
        roll_deg = 0.0; pitch_deg = 0.0; yaw_deg = 0.0;

        //reset object2
        px2 = 10.0; py2 = 0.0; pz2 = 10.0;
        vx2 = 0; vy2 = 0; vz2 = 0;
        vel2= 0;
        q_w2 = 1.0; q_x2 = 0.0; q_y2 = 0.0; q_z2 = 0.0;
        omega_x2 = 0.0; omega_y2 = 0.0; omega_z2 = 0.0;
        roll_deg2 = 0.0; pitch_deg2 = 0.0; yaw_deg2 = 0.0;

        object2.set_pos(px2, py2, pz2);
        object2.set_ang(roll_deg2, pitch_deg2, yaw_deg2);

        launched = false;

    }

    double calcScale(){

        double test_fx = calc_x(force, start_pitch, start_yaw);
        double test_fy = calc_y(force, start_pitch);
        double test_fz = calc_z(force, start_pitch, start_yaw);

        double test_px = 0, test_py = 0, test_pz = 0;
        double test_vx = 0, test_vy = 0, test_vz = 0; 
        double test_t = 0, test_dt  =  0.1; 
        double test_vel = 0; 
        double test_ax, test_ay, test_az;
        double max_x = 0, max_y = 0, max_z  =  0;
        double test_den;

        while(test_t <= dur){
            test_t += test_dt;
            test_ax = 0.0; test_ay = -9.8; test_az = 0;
            
            if(test_t < bt){
                test_ax += calc_ax(test_fx, mass);
                test_ay += calc_ay(test_fy, mass) + 9.8;
                test_az += calc_az(test_fz, mass);

            }
            test_vel = sqrt(test_vx * test_vx + test_vy * test_vy + test_vz * test_vz);
       
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
                test_ax -= (drag / mass) * (test_vx / test_vel);
                test_ay -= (drag / mass) * (test_vy / test_vel);
                test_az -= (drag / mass) * (test_vz / test_vel);

            }
            
            test_vx += test_ax * test_dt; 
            test_vy += test_ay * test_dt; 
            test_vz += test_az * test_dt;
            test_px += test_vx * test_dt; 
            test_py += test_vy * test_dt; 
            test_pz += test_vz * test_dt;

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
        
        //forces computed from current attitude + air-relative vel
        //body axis comes from the quaternion
        //+Y = object forward axis in body space, identity quaternion points "up"
        double bx, by, bz;
        quatRotate(q_w, q_x, q_y, q_z, 0.0, 1.0, 0.0, bx, by, bz);
        const double bmag = sqrt(bx * bx + by * by + bz * bz);
        if(bmag > 0.0){ bx /= bmag; by /= bmag; bz /= bmag; }

        //air-relative velocity, drag and AoA use this
        const double vrel_x = vx - wind_x;
        const double vrel_y = vy - wind_y;
        const double vrel_z = vz - wind_z;
        const double vrel = sqrt(vrel_x * vrel_x + vrel_y * vrel_y + vrel_z * vrel_z);
        vel = sqrt((vx * vx) + (vy * vy) + (vz * vz)); //UI/debug

        //thrust points along the body axis during motor burn
        //replaced old fixed fx/fy/fz from launch angle
        if(t < bt){

            const double thrust = force;
            cur_ax += thrust * bx / mass;
            cur_ay += thrust * by / mass;
            cur_az += thrust * bz / mass;
            
        }
        
        double tor_x = 0.0, tor_y = 0.0, tor_z = 0.0;   // CHECK this may be more complex than this
        double drag = 0.0;
        
        if(vrel > 0){
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

                //drag magnitude uses air-relative speed, then applies opposite to v_rel direction
                drag = calc_drag(vrel, airden, dcoef, area);
                const double vhat_x = vrel_x / vrel;
                const double vhat_y = vrel_y / vrel;
                const double vhat_z = vrel_z / vrel;

                const double drag_fx = -drag * vhat_x;
                const double drag_fy = -drag * vhat_y;
                const double drag_fz = -drag * vhat_z;

                cur_ax += drag_fx / mass;
                cur_ay += drag_fy / mass;
                cur_az += drag_fz / mass;

                //stable alignment torque
                //apply torque that rotates nose toward airflow direction
                const double cx = by * vhat_z - bz * vhat_y;
                const double cy = bz * vhat_x - bx * vhat_z;
                const double cz = bx * vhat_y - by * vhat_x;
                const double sin_alpha = sqrt(cx*cx + cy*cy + cz*cz);

                const double qbarA = 0.5 * airden * vrel * vrel * area;
                const double k_align = qbarA * cn_alpha;

                tor_x += k_align * cx;
                tor_y += k_align * cy;
                tor_z += k_align * cz;

                //angular damping as a torque (opposes rotation).
                tor_x -= ang_damp * omega_x;
                tor_y -= ang_damp * omega_y;
                tor_z -= ang_damp * omega_z;
        }

        vx += cur_ax * dt;
        vy += cur_ay * dt;
        vz += cur_az * dt;
        px += vx * dt;
        py += vy * dt;
        pz += vz * dt;
        
        //ang accel to ang vel
        omega_x += (tor_x / moi) * dt;
        omega_y += (tor_y / moi) * dt;
        omega_z += (tor_z / moi) * dt;
        
        //quaternion from ang vel
        double dq_w = 0.5 * (-q_x * omega_x - q_y * omega_y - q_z * omega_z);
        double dq_x = 0.5 * (q_w * omega_x + q_y * omega_z - q_z * omega_y);
        double dq_y = 0.5 * (q_w * omega_y - q_x * omega_z + q_z * omega_x);
        double dq_z = 0.5 * (q_w * omega_z + q_x * omega_y - q_y * omega_x);
        
        q_w += dq_w * dt;
        q_x += dq_x * dt;
        q_y += dq_y * dt;
        q_z += dq_z * dt;
        
        //normalize quaternion
        double q_mag = sqrt(q_w * q_w + q_x * q_x + q_y * q_y + q_z * q_z);
        if(q_mag > 0.0){
            q_w /= q_mag;
            q_x /= q_mag;
            q_y /= q_mag;
            q_z /= q_mag;
        }
        
        //convert Euler angle
        EulerAngles angles = ToEulerAngles(q_w, q_x, q_y, q_z);
        roll_deg = angles.roll * 57.2958;     // rad to deg
        pitch_deg = angles.pitch * 57.2958;
        yaw_deg = angles.yaw * 57.2958;

        object1.set_pos(px, py, pz);
        object1.set_ang(roll_deg, pitch_deg, yaw_deg);

        //calculate distance to object1
        const double to_obj1_x = px - px2;
        const double to_obj1_y = py - py2;
        const double to_obj1_z = pz - pz2;
        const double dist_to_obj1_check = sqrt(to_obj1_x * to_obj1_x + to_obj1_y * to_obj1_y + to_obj1_z * to_obj1_z);

        //object2 physics update
        if(!launched || (py2 <= 0 && t > dt)) {
            //object2 on ground
        } else {

            double cur_ax2 = 0.0, cur_ay2 = -9.8, cur_az2 = 0.0;

            //object2 body axis from quaternion
            double bx2, by2, bz2;
            quatRotate(q_w2, q_x2, q_y2, q_z2, 0.0, 1.0, 0.0, bx2, by2, bz2);
            const double bmag2 = sqrt(bx2 * bx2 + by2 * by2 + bz2 * bz2);
            if(bmag2 > 0.0){ bx2 /= bmag2; 
                by2 /= bmag2; bz2 /= bmag2;
            }

            //air-relative velocity for object2
            const double vrel_x2 = vx2 - wind_x;
            const double vrel_y2 = vy2 - wind_y;
            const double vrel_z2 = vz2 - wind_z;
            const double vrel2 = sqrt(vrel_x2 * vrel_x2 + vrel_y2 * vrel_y2 + vrel_z2 * vrel_z2);
            vel2 = sqrt((vx2 * vx2) + (vy2 * vy2) + (vz2 * vz2));

            //thrust for object2 during burn
            if(t < bt){

                const double thrust = force;
                cur_ax2 += thrust * bx2 / mass;
                cur_ay2 += thrust * by2 / mass;
                cur_az2 += thrust * bz2 / mass;

            }

            double tor_x2 = 0.0, tor_y2 = 0.0, tor_z2 = 0.0;
            double drag2 = 0.0;

        if(vrel2 > 0){
            if(py2 >= 90000)
                            airden = 0;
                    else if(py2 >= 84852)
                            airden = 0.00001;
                    else if(py2 >= 70000) 
                            airden = 0.00008;
                    else if(py2 >= 60000) 
                            airden = 0.0003;
                    else if(py2 >= 50000) 
                            airden = 0.001;
                    else if(py2 >= 40000) 
                            airden = 0.004;
                    else if(py2 >= 30000) 
                            airden = 0.018;
                    else if(py2 >= 20000) 
                            airden = 0.088;
                    else if(py2 >= 15000) 
                            airden = 0.195;
                    else if(py2 >= 11000) 
                            airden = 0.364;
                    else if(py2 >= 10000) 
                            airden = 0.413;
                    else if(py2 >= 5000) 
                            airden = 0.736;
                    else if(py2 >= 2000) 
                            airden = 1.007;
                    else if (py2 >=1000) 
                            airden = 1.112;
                    else airden = 1.225;


                drag2 = calc_drag(vrel2, airden, dcoef, area);
                const double vhat_x2 = vrel_x2 / vrel2;
                const double vhat_y2 = vrel_y2 / vrel2;
                const double vhat_z2 = vrel_z2 / vrel2;

                const double drag_fx2 = -drag2 * vhat_x2;
                const double drag_fy2 = -drag2 * vhat_y2;
                const double drag_fz2 = -drag2 * vhat_z2;

                cur_ax2 += drag_fx2 / mass;
                cur_ay2 += drag_fy2 / mass;
                cur_az2 += drag_fz2 / mass;

                //stable alignment torque for object2
                const double cx2 = by2 * vhat_z2 - bz2 * vhat_y2;
                const double cy2 = bz2 * vhat_x2 - bx2 * vhat_z2;
                const double cz2 = bx2 * vhat_y2 - by2 * vhat_x2;
                const double sin_alpha2 = sqrt(cx2*cx2 + cy2*cy2 + cz2*cz2);

                const double qbarA2 = 0.5 * airden * vrel2 * vrel2 * area;
                const double k_align2 = qbarA2 * cn_alpha;

                tor_x2 += k_align2 * cx2;
                tor_y2 += k_align2 * cy2;
                tor_z2 += k_align2 * cz2;

                //angular damping for object2
                tor_x2 -= ang_damp * omega_x2;
                tor_y2 -= ang_damp * omega_y2;
                tor_z2 -= ang_damp * omega_z2;
            }

            //pursuit guidance
            if(t >= pursuit_delay){
                //guide object2 toward object1
                if(dist_to_obj1_check > 0.1){  //only pursue if not too close
                    double desired_x = to_obj1_x / dist_to_obj1_check;
                    double desired_y = to_obj1_y / dist_to_obj1_check;
                    double desired_z = to_obj1_z / dist_to_obj1_check;

                    //create quaternion pointing toward object1
                    const glm::vec3 from(0.0f, 1.0f, 0.0f);
                    const glm::vec3 desired_dir((float)desired_x, (float)desired_y, (float)desired_z);
                    const float d = glm::clamp(glm::dot(from, desired_dir), -1.0f, 1.0f);

                    glm::quat desired_quat;
                    
                    if(d < -0.9999f){
                        
                        desired_quat = glm::quat(0.0f, 1.0f, 0.0f, 0.0f);
                    
                        } else {
                        
                        const glm::vec3 c = glm::cross(from, desired_dir);
                        desired_quat = glm::normalize(glm::quat(1.0f + d, c.x, c.y, c.z));

                    }

                    //smooth path toward orientation
                    glm::quat current_q2((float)q_w2, (float)q_x2, (float)q_y2, (float)q_z2);
                    if(glm::dot(current_q2, desired_quat) < 0.0f) desired_quat = -desired_quat;

                    float slerp_factor = 1.0f - expf(-pursuit_turn_rate * dt);
                    glm::quat new_q2 = glm::normalize(glm::slerp(current_q2, desired_quat, slerp_factor));
                    q_w2 = new_q2.w; q_x2 = new_q2.x; q_y2 = new_q2.y; q_z2 = new_q2.z;

                    //apply guidance acceleration
                    const double guidance_strength = 20.0;  //m/s^2
                    cur_ax2 += guidance_strength * desired_x;
                    cur_ay2 += guidance_strength * desired_y;
                    cur_az2 += guidance_strength * desired_z;

                }
            }

            vx2 += cur_ax2 * dt;
            vy2 += cur_ay2 * dt;
            vz2 += cur_az2 * dt;
            px2 += vx2 * dt;
            py2 += vy2 * dt;
            pz2 += vz2 * dt;

            //ang accel to ang vel for object2
            omega_x2 += (tor_x2 / moi) * dt;
            omega_y2 += (tor_y2 / moi) * dt;
            omega_z2 += (tor_z2 / moi) * dt;

            //quaternion from ang vel for object2
            double dq_w2 = 0.5 * (-q_x2 * omega_x2 - q_y2 * omega_y2 - q_z2 * omega_z2);
            double dq_x2 = 0.5 * (q_w2 * omega_x2 + q_y2 * omega_z2 - q_z2 * omega_y2);
            double dq_y2 = 0.5 * (q_w2 * omega_y2 - q_x2 * omega_z2 + q_z2 * omega_x2);
            double dq_z2 = 0.5 * (q_w2 * omega_z2 + q_x2 * omega_y2 - q_y2 * omega_x2);

            q_w2 += dq_w2 * dt;
            q_x2 += dq_x2 * dt;
            q_y2 += dq_y2 * dt;
            q_z2 += dq_z2 * dt;

            //normalize quaternion for object2
            double q_mag2 = sqrt(q_w2 * q_w2 + q_x2 * q_x2 + q_y2 * q_y2 + q_z2 * q_z2);
            if(q_mag2 > 0.0){
                q_w2 /= q_mag2;
                q_x2 /= q_mag2;
                q_y2 /= q_mag2;
                q_z2 /= q_mag2;
            }

            //convert euler angle for object2
            EulerAngles angles2 = ToEulerAngles(q_w2, q_x2, q_y2, q_z2);
            roll_deg2 = angles2.roll * 57.2958;
            pitch_deg2 = angles2.pitch * 57.2958;
            yaw_deg2 = angles2.yaw * 57.2958;

            object2.set_pos(px2, py2, pz2);
            object2.set_ang(roll_deg2, pitch_deg2, yaw_deg2);
        }
        static int debug_count = 0;
        if(debug_count++ % 10 == 0){

                double nbx, nby, nbz;
                quatRotate(q_w, q_x, q_y, q_z, 0.0, 1.0, 0.0, nbx, nby, nbz);
                const double nmag = sqrt(nbx*nbx + nby*nby + nbz*nbz);
                if(nmag > 0.0){ nbx/=nmag; nby/=nmag; nbz/=nmag; }

                double vhatx = 0.0, vhaty = 0.0, vhatz = 0.0;
                if(vel > 1e-6){ vhatx = vx/vel; vhaty = vy/vel; vhatz = vz/vel; }

                std::cout << "t=" << t << " py=" << py << " vy=" << vy << " roll=" << roll_deg 
                      << " pitch=" << pitch_deg << " yaw=" << yaw_deg << std::endl;
                std::cout << " nose=(" << nbx << "," << nby << "," << nbz << ")"
                          << " vhat=(" << vhatx << "," << vhaty << "," << vhatz << ")"
                          << " launch=(" << launch_dir_x << "," << launch_dir_y << "," << launch_dir_z << ")"
                          << std::endl;
        }
    }
};