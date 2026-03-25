#ifndef objects_h
#define objects_h
#include <iostream>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

struct vec1{
    float x, y, z;
};

struct EulerAngles{
    double roll, pitch, yaw;
};

class rocket{

    public:

    void init_rocket (float start_pitch, float start_yaw){

        pos.x = 0.0f;
        pos.y = 0.0f;
        pos.z = 0.0f;
        
        rot.y = start_yaw;
        rot.z = start_pitch;

    }

    void set_pos(float new_x, float new_y, float new_z){

        pos.x = new_x;
        pos.y = new_y;
        pos.z = new_z;

    };

    void set_ang(float new_ang_x, float new_ang_y, float new_ang_z){

        rot.x = new_ang_x;
        rot.y = new_ang_y;
        rot.z = new_ang_z;

    }

    vec1 get_pos(){
        return pos;
    }

    
    glm::mat4 get_model_matrix() {

        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, glm::vec3(pos.x, pos.y, pos.z)); //trans to world pos
        model = glm::rotate(model, glm::radians(rot.x), glm::vec3(1.0f, 0.0f, 0.0f)); //rotates around (x,y,z)
        model = glm::rotate(model, glm::radians(rot.y), glm::vec3(0.0f, 1.0f, 0.0f));
        model = glm::rotate(model, glm::radians(rot.z), glm::vec3(0.0f, 0.0f, 1.0f));
        return model;

    }


    private:

    vec1 pos;
    vec1 rot;

};

#endif //object_h