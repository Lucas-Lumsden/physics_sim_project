#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "rocket_sim/sim.h"
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
    
    SimState sim;
    sim.init(sim.airden, sim.start_ang, sim.thrust_ang, sim.aero_ang, sim.gyro_ang, sim.ang);

    std::ifstream load("C:\\Users\\lucge\\physics_sim_project\\sim_params.txt");
    if(load.is_open()){
        load >> sim.mass >> sim.force >> sim.start_ang >> sim.bt >> sim.dcoef >> sim.area;
        load.close();
    }

//find scale
    double scale = sim.calcScale();

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

        ImGui::InputFloat("Mass (kg)", &sim.mass, 1.0f, 100.0f);
        ImGui::InputFloat("Force (N)", &sim.force, 1.0f, 2000.0f);
        ImGui::InputFloat("Angle (deg)", &sim.start_ang, 1.0f, 89.0f);
        ImGui::InputFloat("Burn time (s)", &sim.bt, 0.1f, 20.0f);
        ImGui::InputFloat("Drag coef", &sim.dcoef, 0.01f, 1.0f);
        ImGui::InputFloat("Frontal area (kg/m^3)", &sim.area, 0.01f, 5.0f);

        ImGui::Spacing();

        if(ImGui::Button("Launch")){
            sim.launch();
            scale = sim.calcScale();

        }
        ImGui::SameLine();
        if(ImGui::Button("Reset")){
            sim.reset();
            path.clear();
        }

        ImGui::Spacing();
        ImGui::Separator();
        ImGui::Text("-- Live Data --");
        ImGui::Text("Time: %.2f s", sim.t);
        ImGui::Text("X: %.2f m", sim.px);
        ImGui::Text("Y: %.2f m", sim.py);
        ImGui::Text("Vel: %.2f m/s", sim.vel);
        ImGui::Text("Vx: %.2f m/s", sim.vx);
        ImGui::Text("Vy: %.2f m/s", sim.vy);

        ImGui::End();

        sim.step();

        if(sim.launched){
            float sx = (float)(sim.px/scale)*1.8f - 0.9f;
            float sy = (float)(sim.py/scale)*1.8f - 0.9f;
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
            
            glRotatef((float)sim.ang, 0.0f, 0.0f, 1.0f); //z=1 determines rotation occurs around z axis

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