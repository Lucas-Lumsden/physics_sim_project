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

int main(){

    float mass, force, angle, bt, dcoef, area;
    std::ifstream load("C:\\Users\\lucge\\physics_sim_project\\sim_params.txt");
    if(load.is_open()){
        load >> mass >> force >> angle >> bt >> dcoef >> area;
        load.close();
    }

    double fx = calc_x(force, angle);
    double fy = calc_y(force, angle);
    double t = 0.0, dt = 0.01, dur = 60.0;
    double vx = 0.0, vy = 0.0, vel = 0.0;
    double px = 0.0, py = 0.0;
    double test_dt = 0.1;
    bool launched = false;

    //find scale
    auto calcScale = [&]() {
        double test_px=0, test_py=0, test_vx=0, test_vy=0;
        double test_t=0, test_vel=0, test_ax, test_ay;
        double max_x=0, max_y=0;
        while(test_t <= dur){
            test_t += test_dt;
            test_ax = 0.0; test_ay = -9.8;
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
            test_vx += test_ax*test_dt; test_vy += test_ay*test_dt;
            test_px += test_vx*test_dt; test_py += test_vy*test_dt;
            if(test_px > max_x) max_x = test_px;
            if(test_py > max_y) max_y = test_py;
            if(test_py < 0 && test_dt > test_dt) break;
        }
        return std::max(max_x, max_y) * 1.2;
    };

    double scale = calcScale();

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
        ImGui::InputFloat("Angle (deg)", &angle, 1.0f, 89.0f);
        ImGui::InputFloat("Burn time (s)", &bt, 0.1f, 20.0f);
        ImGui::InputFloat("Drag coef", &dcoef, 0.01f, 1.0f);
        ImGui::InputFloat("Frontal area (kg/m^3)", &area, 0.01f, 5.0f);

        ImGui::Spacing();

        if(ImGui::Button("Launch")){
            t=0; vx=0; vy=0; vel=0; px=0; py=0;
            path.clear();
            launched = true;
            fx = calc_x(force, angle);
            fy = calc_y(force, angle);
            scale = calcScale();
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
                double drag = calc_drag(vel, 1.225, dcoef, area);
                cur_ax -= (drag/mass)*(vx/vel);
                cur_ay -= (drag/mass)*(vy/vel);
            }
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
            glBegin(GL_TRIANGLES);
                glColor3f(1.0f, 0.0f, 0.0f);
                glVertex2f(sx, sy + size*2);
                glVertex2f(sx - size, sy - size);
                glVertex2f(sx + size, sy - size);
            glEnd();
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