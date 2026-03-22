#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "rocket_sim/objects.h"
#include "rocket_sim/math.h"
#include "imgui/imgui.h"
#include "imgui/imgui_impl_glfw.h"
#include "imgui/imgui_impl_opengl3.h"
#include <vector>
#include <iostream>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

int main(){

    std::cout << "starting" << std::endl;
    
    double airden;
    float start_ang;
    float thrust_ang = 0.0f;
    float aero_ang = 0.0f;          //place holders until i can determine how to calc the forces
    float gyro_ang = 0.0f;
    double ang = calc_ang(thrust_ang, aero_ang, gyro_ang, start_ang);
    double rocket_ang_deg_ps = 10;    
    float mass, force, bt, dcoef, area;

    std::ifstream load("C:\\Users\\lucge\\physics_sim_project\\sim_params.txt");
    if(load.is_open()){
        load >> mass >> force >> start_ang >> bt >> dcoef >> area;
        load.close();
    }
    
    double fx = calc_x(force, ang);
    double fy = calc_y(force, ang);
    double t = 0.0, dt = 0.01, dur = 600.0;
    double vx = 0.0, vy = 0.0, vel = 0.0;
    double px = 0.0, py = 0.0;
    double test_dt = 0.1;                          
    bool launched = false;

    //find scale
    auto calcScale = [&]() {

        double test_fx = calc_x(force, start_ang);
        double test_fy = calc_y(force, start_ang);


        double test_px=0, test_py=0, test_vx=0, test_vy=0;
        double test_t=0, test_vel=0, test_ax, test_ay;
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
    };

    double scale = calcScale();

std::cout << "scale done, scale=" << scale << std::endl;

//window
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);

    GLFWwindow* window = glfwCreateWindow(1200, 700, "Rocket Sim", NULL, NULL);
    if(!window){ glfwTerminate(); return -1; }
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

//imgui begins
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");
    ImGui::StyleColorsDark();

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

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(900, 10), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(290, 420), ImGuiCond_Always);
        ImGui::Begin("Rocket Parameters");

        ImGui::InputFloat("Mass (kg)", &mass, 1.0f, 100.0f);
        ImGui::InputFloat("Force (N)", &force, 1.0f, 2000.0f);
        ImGui::InputFloat("Angle (deg)", &start_ang, 1.0f, 89.0f);
        ImGui::InputFloat("Burn time (s)", &bt, 0.1f, 20.0f);
        ImGui::InputFloat("Drag coef", &dcoef, 0.01f, 1.0f);
        ImGui::InputFloat("Frontal area (kg/m^3)", &area, 0.01f, 5.0f);

        ImGui::Spacing();

        if(ImGui::Button("Launch")){
            t=0; vx=0; vy=0; vel=0; px=0; py=0;
            path.clear();
            launched = true;
            fx = calc_x(force, start_ang);
            fy = calc_y(force, start_ang);
            ang = start_ang;
            scale = calcScale();
            std::cout << "scale=" << scale 
              << " mass=" << mass 
              << " force=" << force 
              << " start_ang=" << start_ang 
              << " bt=" << bt << std::endl;
        }
        ImGui::SameLine();
        if(ImGui::Button("Reset")){
            t=0; vx=0; vy=0; vel=0; px=0; py=0;
            path.clear(); launched=false;
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("-- Live Data --");
        ImGui::Text("Time: %.2f s", t);
        ImGui::Text("X: %.2f m", px);
        ImGui::Text("Y: %.2f m", py);
        ImGui::Text("Vel: %.2f m/s", vel);
        ImGui::Text("Vx: %.2f m/s", vx);
        ImGui::Text("Vy: %.2f m/s", vy);

        ImGui::End();


        if(launched && (py >= 0 || t == 0)){
            t += dt;
            double cur_ax = 0.0, cur_ay = -9.8;
            if(t < bt){
                cur_ax += calc_ax(fx, mass);
                cur_ay += calc_ay(fy, mass) + 9.8;
            }
            vel = sqrt(vx*vx + vy*vy);
            
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
                        airden = 1.225;

                double drag = calc_drag(vel, airden, dcoef, area);
                cur_ax -= (drag/mass)*(vx/vel);
                cur_ay -= (drag/mass)*(vy/vel);
            }
            
//rotational calculations
            double ideal_ang_deg = calc_ideal_ang_deg(vx,vy);
            double ang_err = calc_ang_err(ideal_ang_deg, ang);
            double ang_vel_deg_ps = calc_vel_deg_ps(ang_err); // speed of rotation adjusted by gain (0.5 in math.cpp)
            
            if (fabs(ang_vel_deg_ps) > 90.0f){
                ang_vel_deg_ps = copysign(90.0f, ang_vel_deg_ps); // max rotation speed
            }

            ang += ang_vel_deg_ps * dt;

            vx += cur_ax*dt;
            vy += cur_ay*dt;
            px += vx*dt;
            py += vy*dt;

            float sx = (float)(px/scale)*1.8f - 0.9f;
            float sy = (float)(py/scale)*1.8f - 0.9f;
            path.push_back(sx);
            path.push_back(sy);

        }

//ground
        glBegin(GL_LINES);
            glColor3f(1.0f, 1.0f, 1.0f);
            glVertex2f(-0.9f, -0.9f);
            glVertex2f(0.9f, -0.9f);
        glEnd();

// y axis
        glBegin(GL_LINES);
            glColor3f(1.0f, 1.0f, 1.0f);
            glVertex2f(-0.9f, -0.9f);
            glVertex2f(-0.9f, 0.9f);
        glEnd();

//path
        glBegin(GL_LINE_STRIP);
            glColor3f(0.0f, 1.0f, 0.0f);
            for(int i = 0; i < (int)path.size(); i += 2)
                glVertex2f(path[i], path[i+1]);
        glEnd();

        if(!path.empty()){

            float sx = path[path.size()-2];
            float sy = path[path.size()-1];
            float size = 0.02f;

            glPushMatrix();
            glTranslatef(sx, sy, 0.0f); // when expand to 3d, add sz
            
            glRotatef((float)ang, 0.0f, 0.0f, 1.0f); //z=1 determines rotation occurs around z axis

            glBegin(GL_TRIANGLES);
                glColor3f(1.0f, 0.0f, 0.0f);
                glVertex2f(0.0f, size*2);
                glVertex2f(-size, - size);
                glVertex2f(size, - size);
            glEnd();

            glPopMatrix();
        }

//imgui render
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        glfwTerminate();

    return 0;
}