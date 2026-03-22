#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "rocket_sim/objects.h"
#include "rocket_sim/math.h"
#include <vector>
#include <iostream>
#include <cmath>
#include <fstream>
#include <algorithm>

int main(){

    double mass, force, angle, bt, dcoef, area;
    std::ifstream load("C:\\Users\\lucge\\physics_sim_project\\sim_params.txt");
    if(load.is_open()){
        load >> mass >> force >> angle >> bt >> dcoef >> area;
        load.close();
    } else {
        std::cout << "No saved params, enter manually:" << std::endl;
        std::cout << "mass: "; std::cin >> mass;
        std::cout << "force: "; std::cin >> force;
        std::cout << "angle: "; std::cin >> angle;
        std::cout << "bt: "; std::cin >> bt;
        std::cout << "dcoef: "; std::cin >> dcoef;
        std::cout << "area: "; std::cin >> area;
    }

    double fx = calc_x(force, angle);
    double fy = calc_y(force, angle);
    double t = 0.0, dt = 0.01, dur = 60.0;
    double vx = 0.0, vy = 0.0, vel = 0.0;
    double px = 0.0, py = 0.0;

    //window scale finder
    double test_px = 0, test_py = 0;
    double test_vx = 0, test_vy = 0;
    double test_t = 0, test_vel = 0;
    double max_x = 0, max_y = 0;
    double test_ax, test_ay;

    while(test_t <= dur){
        test_t += dt;

        test_ax = 0.0;
        test_ay = -9.8;
        if(test_t < bt){
            test_ax += calc_ax(fx, mass);
            test_ay += calc_ay(fy, mass) + 9.8;
        }
        test_vel = sqrt(test_vx*test_vx + test_vy*test_vy);
        if(test_vel > 0){
            double drag = calc_drag(test_vel, 1.225, dcoef, area);
            test_ax -= (drag/mass)*(test_vx/test_vel);
            test_ay -= (drag/mass)*(test_vy/test_vel);
        }
        test_vx += test_ax * dt;
        test_vy += test_ay * dt;
        test_px += test_vx * dt;
        test_py += test_vy * dt;
        if(test_px > max_x) max_x = test_px;
        if(test_py > max_y) max_y = test_py;
        if(test_py < 0 && test_t > dt) break;
    }

    double scale = std::max(max_x, max_y) * 1.2;

    //window
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);

    GLFWwindow* window = glfwCreateWindow(1600, 1200, "Rocket Sim", NULL, NULL);
    if(!window){
        std::cout << "Window failed" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

    std::vector<float> path;
    double lastTime = glfwGetTime();
    double frameDelay = 1.0/240.0;

    //render
    while(!glfwWindowShouldClose(window)){

        double now = glfwGetTime();
        if(now - lastTime < frameDelay){
            glfwPollEvents();
            continue;
        }
        lastTime = now;

        glClear(GL_COLOR_BUFFER_BIT);

        if(py >= 0 || t == 0){
            t += dt;

            double cur_ax = 0.0;
            double cur_ay = -9.8;
            if(t < bt){
                cur_ax += calc_ax(fx, mass);
                cur_ay += calc_ay(fy, mass) + 9.8;
            }
            vel = sqrt(vx*vx + vy*vy);
            if(vel > 0){
                double drag = calc_drag(vel, 1.225, dcoef, area);
                cur_ax -= (drag/mass) * (vx/vel);
                cur_ay -= (drag/mass) * (vy/vel);
            }
            vx += cur_ax * dt;
            vy += cur_ay * dt;
            px += vx * dt;
            py += vy * dt;

            float sx = (float)(px/scale) * 1.8f - 0.9f;
            float sy = (float)(py/scale) * 1.8f - 0.9f;
            path.push_back(sx);
            path.push_back(sy);
        }

        //path
        glBegin(GL_LINE_STRIP);
            glColor3f(0.0f, 1.0f, 0.0f);
            for(int i = 0; i < (int)path.size(); i += 2)
                glVertex2f(path[i], path[i+1]);
        glEnd();

        //object
        glPointSize(10.0f);
        glBegin(GL_POINTS);
            glColor3f(1.0f, 0.0f, 0.0f);
            float sx = path.empty() ? -0.9f : path[path.size()-2];
            float sy = path.empty() ? -0.9f : path[path.size()-1];
            glVertex2f(sx, sy);
        glEnd();



        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}