#include <igl/edges.h>
#include "Simulation.h"
#include "Table.h"
#include "Ball.h"
#include "Paddle.h"
#include "CollisionDetection.h"

using namespace std;

class PinballSim : public Simulation {
public:
    PinballSim() : Simulation(), m_collisionDetection(m_objects) { init(); }

    float m_eps = 1.0;

    // CONFIGS
    float m_dt = 1e-2;
    Eigen::Vector3d m_gravity;
    long score = 0;
    //


    // SCENE OBJECS
    Table* p_table;
    Ball* p_ball;
    Paddle* p_paddle_l;
    Paddle* p_paddle_r;

    virtual void init() override {
        m_objects.clear();
        m_objects.push_back(Table());
        p_table = (Table*)&m_objects.back();

        m_objects.push_back(Ball());
        p_ball = (Ball*)&m_objects.back();

        m_objects.push_back(Paddle()); // m_objects[2]
        p_paddle_r = (Paddle*)&m_objects.back();

        //m_objects.push_back(Paddle()); // m_objects[3]
        //p_paddle_r = (Paddle*)&m_objects.back();

        m_collisionDetection.setObjects(m_objects);

        m_gravity << 0, -9.81, 0;


        setTimestep(m_dt);
        reset();
    }

    virtual void resetMembers() override {
        for (auto &o : m_objects) {
            o.reset();
        }

        m_objects[0].setScale(1);
        m_objects[0].setType(ObjType::STATIC);
        m_objects[0].setPosition(Eigen::Vector3d(0, 0, 0));

        Eigen::Matrix3d rotation_y;
        float theta = -1.5; // rotate table back 45°
        rotation_y << cos(theta), 0, sin(theta),
                0, 1, 0,
                -sin(theta), 0, cos(theta);

        Eigen::Matrix3d rotation_x;
        theta = -0.5; // tilt table 30°
        rotation_x << std::cos(theta), -std::sin(theta), 0,
                std::sin(theta), std::cos(theta) ,0,
                0, 0, 1;


        m_objects[0].setRotation(rotation_y*(rotation_x));

        //p_table->setType(ObjType::STATIC);
        m_objects[1].setScale(0.005);
        m_objects[1].setPosition(Eigen::Vector3d(0, 1, 0));
        m_objects[1].setMass(1);

        Eigen::MatrixXd color(1, 3);
        color << 0.0, 204.0 / 255.0, 102.0 / 255.0;
        m_objects[1].setColors(color);

        m_objects[2].setScale(0.2);
        m_objects[2].setPosition(Eigen::Vector3d(1.7, -3.2, 5.7));
        
        m_objects[2].setRotation(rotation_y*(rotation_x));
    }

    virtual void updateRenderGeometry() override {
        for (size_t i = 0; i < m_objects.size(); i++) {
            RigidObject &o = m_objects[i];
            if (o.getID() < 0) {
                m_renderVs.emplace_back();
                m_renderFs.emplace_back();
            }

            m_objects[i].getMesh(m_renderVs[i], m_renderFs[i]);
        }
    }

    virtual bool advance();

    virtual void renderRenderGeometry(igl::opengl::glfw::Viewer &viewer) override {

        for (size_t i = 0; i < m_objects.size(); i++) {
            RigidObject &o = m_objects[i];
            if (o.getID() < 0) {
                int new_id = 0;
                if (i > 0) {
                    new_id = viewer.append_mesh();
                    o.setID(new_id);
                } else {
                    o.setID(new_id);
                }

                size_t meshIndex = viewer.mesh_index(o.getID());
                viewer.data_list[meshIndex].show_lines = false;
                viewer.data_list[meshIndex].set_face_based(true);
                viewer.data_list[meshIndex].point_size = 2.0f;
                viewer.data_list[meshIndex].clear();
            }
            size_t meshIndex = viewer.mesh_index(o.getID());

            viewer.data_list[meshIndex].set_mesh(m_renderVs[i], m_renderFs[i]);
            viewer.data_list[meshIndex].compute_normals();

            Eigen::MatrixXd color;
            o.getColors(color);
            viewer.data_list[meshIndex].set_colors(color);
        }

    }

#pragma region SettersAndGetters

#pragma endregion SettersAndGetters

private:
    std::vector<Eigen::MatrixXd> m_renderVs;  // vertex positions for rendering
    std::vector<Eigen::MatrixXi> m_renderFs;  // face indices for rendering
    CollisionDetection m_collisionDetection;
};