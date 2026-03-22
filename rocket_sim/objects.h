#include <iostream>
#include <cmath>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

struct vec1{
    float x, y, z;
};

class rocket{

    public:

    void init_rocket (float start_pos_x, float start_pos_y, float start_rot_z){

        pos.x = start_pos_x;
        pos.y = start_pos_y;
        rot.z = start_rot_z;

    }

    void set_pos(float new_x, float new_y){

        pos.x = new_x;
        pos.y = new_y;

    };

    void set_ang(float new_ang){

        rot.z += new_ang;

    }

    glm::mat4 get_model_matrix() {

        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, glm::vec3(pos.x, pos.y, 1.0f)); //trans to world pos
        model = glm::rotate(model, glm::radians(rot.z), glm::vec3(0.0f, 0.0f, 1.0f)); //rotates around (x,y,z)
        return model;

    }

    vec1 get_pos(){
        return pos;
    }
    vec1 get_rot(){
        return rot;
    }

    private:

    vec1 acc;
    vec1 pos;
    vec1 rot;

    float heading;

};