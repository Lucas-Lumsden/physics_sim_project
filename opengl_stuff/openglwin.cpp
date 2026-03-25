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

static bool orbiting = false;
static double lastx = 0.0f, lasty = 0.0;
static float camYaw = 45.0f;
static float camPitch = 20.0f;
static float camRadius = 5.0f;

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos){
    
    if(!orbiting) return;
    if(ImGui::GetIO().WantCaptureMouse) return;

    float sensitivity = 0.25f;
    float xoffset = float(xpos - lastx);
    float yoffset = float(ypos - lasty);

    lastx= xpos;
    lasty = ypos;

    camYaw += xoffset * sensitivity;
    camPitch += -yoffset * sensitivity;
    camPitch = glm::clamp(camPitch, -89.0f, 89.0f);

};

void mouse_button_callback(GLFWwindow * window, int button, int action, int mods){
    if(ImGui::GetIO().WantCaptureMouse) return;

    if(button == GLFW_MOUSE_BUTTON_LEFT){
        orbiting = (action == GLFW_PRESS);
    
        if(orbiting) glfwGetCursorPos(window, &lastx, &lasty);
    };
};

void scroll_callback(GLFWwindow * window, double xoffset, double yoffset){
    if(ImGui::GetIO().WantCaptureMouse) return;

    camRadius = glm::clamp(camRadius - float(yoffset) * 0.3f, 1.0f, 20.0f);
};

int main(){
    
    SimState sim;

    std::ifstream load("C:\\Users\\lucge\\physics_sim_project\\sim_params.txt");
    if(load.is_open()){
        load >> sim.mass >> sim.force >> sim.start_pitch >> sim.start_yaw >> sim.bt >> sim.dcoef >> sim.area;
        load.close();
    } else {
        sim.start_pitch = 90.0f;
        sim.start_yaw = 0.0f;  // default values
    }

    sim.init(sim.mass, sim.force, sim.start_pitch, sim.start_yaw, sim.bt, sim.dcoef, sim.area);

//find scale
    double scale = sim.calcScale();

//window
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);
    glfwWindowHint(GLFW_DEPTH_BITS, 24);

    GLFWwindow* window = glfwCreateWindow(1200, 700, "Rocket Sim", NULL, NULL);
    if(!window){ glfwTerminate(); return -1; }
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetScrollCallback(window, scroll_callback);

    glm::mat4 projection = glm::perspective(glm::radians(45.0f), 1200.0f / 700.0f, 0.1f, 100.0f); // CHECk

    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf(glm::value_ptr(projection));

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

        float yawRad = glm::radians(camYaw);
        float pitchRad = glm::radians(camPitch);

        glm::vec3 target = glm::vec3(-0.9f, -0.9f, 0.0f);
        if(sim.launched){
            float sx = (float)(sim.px/scale)*1.8f - 0.9f;
            float sy = (float)(sim.py/scale)*1.8f - 0.9f;
            float sz = (float)(sim.pz/scale)*1.8f;
            target = glm::vec3(sx, sy, sz);
        }

        glm::vec3 camPos = glm::vec3(
            camRadius * cos(pitchRad) * sin(yawRad),
            camRadius * sin(pitchRad),
            camRadius * cos(pitchRad) * cos(yawRad)
        );

        glm::mat4 view = glm::lookAt(
            camPos + target,
            target,
            glm::vec3(0.0f, 1.0f, 0.0f)
        );

//view
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(view));

        double now = glfwGetTime();
        if(now - lastTime < frameDelay){
            glfwPollEvents();
            continue;
        }
        lastTime = now;

        glEnable(GL_DEPTH_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowPos(ImVec2(900, 10), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(290, 420), ImGuiCond_Always);
        ImGui::Begin("Rocket Parameters");

        ImGui::InputFloat("Mass (kg)", &sim.mass, 0.0f, 0.0f);
        ImGui::InputFloat("Force (N)", &sim.force, 0.0f, 0.0f);
        ImGui::InputFloat("Pitch (deg)", &sim.start_pitch, 0.0f, 0.0f);
        ImGui::InputFloat("Yaw (deg)", &sim.start_yaw, 0.0f, 0.0f);
        //ImGui::InputFloat("Roll (deg)", &sim.roll, 0.0f, 0.0f);   // CHECK if  this makes a diff, then how to do
        ImGui::InputFloat("Burn time (s)", &sim.bt, 0.0f, 0.0f);
        ImGui::InputFloat("Drag coef", &sim.dcoef, 0.0f, 0.0f);
        ImGui::InputFloat("Frontal area (kg/m^3)", &sim.area, 0.0f, 0.0f);

        ImGui::Spacing();

        if(ImGui::Button("Launch")){
            std::cout << "Launch clicked" << std::endl;
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
        ImGui::Text("Z: %.2f m", sim.pz);
        ImGui::Text("Vel: %.2f m/s", sim.vel);
        ImGui::Text("Vx: %.2f m/s", sim.vx);
        ImGui::Text("Vy: %.2f m/s", sim.vy);
        ImGui::Text("Vz: %.2f m", sim.vz);

        ImGui::End();

        sim.step();

        if(sim.launched){
            float sx = (float)(sim.px/scale)*1.8f - 0.9f;
            float sy = (float)(sim.py/scale)*1.8f - 0.9f;
            float sz = (float)(sim.pz/scale)*1.8f;

            path.push_back((float)(sim.px/scale)*1.8f - 0.9f);
            path.push_back((float)(sim.py/scale)*1.8f - 0.9f);
            path.push_back((float)(sim.pz/scale)*1.8f);

        }

// x axis
        glBegin(GL_LINES);
            glColor3f(1.0f, 0.0f, 0.0f);
            glVertex3f(-0.9f, -0.9f, 0.0f);
            glVertex3f(0.9f, -0.9f, 0.0);
        glEnd();

// y axis
        glBegin(GL_LINES);
            glColor3f(0.0f, 1.0f, 0.0f);
            glVertex3f(-0.9f, -0.9f, 0.0f);
            glVertex3f(-0.9f, 0.9f, 0.0f);
        glEnd();

// z axis
        glBegin(GL_LINES);
            glColor3f(0.0f, 0.0f, 1.0f);
            glVertex3f(-0.9f, -0.9f, 0.9f); // end point
            glVertex3f(-0.9f, -0.9f, 0.0f); // start point
        glEnd();

//path
        glBegin(GL_LINE_STRIP);
            glColor3f(1.0f, 1.0f, 0.0f);
            for(int i = 0; i < (int)path.size(); i += 3)
                glVertex3f(path[i], path[i+1], path[i+2]);
        glEnd();

        if(sim.launched){
            float sx = (float)(sim.px/scale)*1.8f - 0.9f;
            float sy = (float)(sim.py/scale)*1.8f - 0.9f;
            float sz = (float)(sim.pz/scale)*1.8f;

            float size = 0.02f;

            glm::quat orientation(

                (float) sim.q_w,
                (float) sim.q_x,
                (float) sim.q_y,
                (float) sim.q_z

            );

                glm::quat currentOrientation;
                float angle = glm::radians(45.0f);
                glm::vec3 axis = glm::vec3(0.0f, 1.0f, 0.0f);

        glPushMatrix();
        
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, glm::vec3(sx, sy, sz));
        model = model * glm::mat4_cast(orientation);
        glMultMatrixf(glm::value_ptr(model));
            
        float nose[] = {0.0f, size*2, 0.0f};    // nose
        float b1[] = {size, -size, size};   // base front right
        float b2[] = {-size, -size, size};  // base front left
        float b3[] = {-size, -size, -size}; // base back left
        float b4[] = {size, -size, -size};  // base back right

        glBegin(GL_TRIANGLES);
            
// front face
            glColor3f(1.0f, 0.0f, 0.0f);
            glVertex3fv(nose);
            glVertex3fv(b1);
            glVertex3fv(b2);

// left face
            glColor3f(0.0f, 1.0f, 0.0f);
            glVertex3fv(nose);
            glVertex3fv(b2);
            glVertex3fv(b3);

// back face
            glColor3f(0.0f, 0.0f, 1.0f);
            glVertex3fv(nose);
            glVertex3fv(b3);
            glVertex3fv(b4);

// right face
            glColor3f(1.0f, 1.0f, 0.0f);
            glVertex3fv(nose);
            glVertex3fv(b4);
            glVertex3fv(b1);

// base triangle 1
            glColor3f(0.5f, 0.5f, 0.5f);
            glVertex3fv(b1);
            glVertex3fv(b3);
            glVertex3fv(b2);

// base triangle 2
            glColor3f(0.5f, 0.5f, 0.5f);
            glVertex3fv(b1);
            glVertex3fv(b4);
            glVertex3fv(b3);

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