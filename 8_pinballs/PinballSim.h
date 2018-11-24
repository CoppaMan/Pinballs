#include <igl/edges.h>
#include "Simulation.h"
#include "Table.h"
#include "Ball.h"
#include "CollisionDetection.h"
#include "Paddle.h"
#include "Score.h"

using namespace std;

class PinballSim : public Simulation {
public:
    PinballSim() : Simulation(), m_collisionDetection(m_objects) { init(); }

    float m_eps = 1.0;

    // CONFIGS
    float m_dt = 1e-2;
    Eigen::Vector3d m_gravity;

    // SCENE OBJECS
    std::shared_ptr<Table> p_table;
    std::shared_ptr<Ball> p_ball;
    std::shared_ptr<Paddle> p_paddle_r;
    std::shared_ptr<Paddle> p_paddle_l;
    std::shared_ptr<Score> score;

    virtual void init() override {
        m_objects.clear();
        p_ball = std::make_shared<Ball>();
        m_objects.emplace_back(p_ball);

        p_table = std::make_shared<Table>();
        m_objects.emplace_back(p_table);

        p_paddle_r = std::make_shared<Paddle>(sf::Keyboard::Key::Right, Eigen::Vector3d(1.7, -3.2, 5.7), true);
        m_objects.emplace_back(p_paddle_r); // Right paddle

        p_paddle_l = std::make_shared<Paddle>(sf::Keyboard::Key::Left, Eigen::Vector3d(-1.7, -3.2, 5.7), false);
        m_objects.emplace_back(p_paddle_l); // Left paddle

        score = std::make_shared<Score>(Eigen::Vector3d(5.75, 4, -4), 8);
        score->emplaceInto(&m_objects); //Add all objects related to the visual score

        m_collisionDetection.setObjects(m_objects);

        m_gravity << 0, -9.81, 0;

        setTimestep(m_dt);
        reset();
    }

    virtual void resetMembers() override {
        for (auto &o : m_objects) {
            o->reset();
        }

        p_table->setScale(1);
        p_table->setType(ObjType::STATIC);
        p_table->setPosition(Eigen::Vector3d(0, 0, 0));

        
        Eigen::Matrix3d rotation_y;
        float theta = -M_PI/2.0; // rotate table back 45°
        rotation_y << cos(theta), 0, sin(theta),
                0, 1, 0,
                -sin(theta), 0, cos(theta);

        Eigen::Matrix3d rotation_x;
        theta = -M_PI/6.0; // tilt table 30°
        rotation_x << std::cos(theta), -std::sin(theta), 0,
                std::sin(theta), std::cos(theta) ,0,
                0, 0, 1;
        

        p_table->setRotation(rotation_y*(rotation_x));

        //p_table->setType(ObjType::STATIC);
        p_ball->setScale(0.005);
        p_ball->setPosition(Eigen::Vector3d(1.5, 0.5, 0));
        p_ball->setMass(1);

        Eigen::MatrixXd color(1, 3);
        color << 0.0, 204.0 / 255.0, 102.0 / 255.0;
        p_ball->setColors(color);

        p_paddle_r->reset_paddle(); //resets the paddle state
        p_paddle_l->reset_paddle();

        score->resetScore();
    }

    virtual void updateRenderGeometry() override {
        for (size_t i = 0; i < m_objects.size(); i++) {
            auto o = m_objects[i];
            if (o->getID() < 0) {
                m_renderVs.emplace_back();
                m_renderFs.emplace_back();
            }

            m_objects[i]->getMesh(m_renderVs[i], m_renderFs[i]);
        }
    }

    virtual bool advance();

    virtual void renderRenderGeometry(igl::opengl::glfw::Viewer &viewer) override {

        for (size_t i = 0; i < m_objects.size(); i++) {
            auto o = m_objects[i];
            if (o->getID() < 0) {
                int new_id = 0;
                if (i > 0) {
                    new_id = viewer.append_mesh();
                    o->setID(new_id);
                } else {
                    o->setID(new_id);
                }

                size_t meshIndex = viewer.mesh_index(o->getID());
                viewer.data_list[meshIndex].show_lines = false;
                viewer.data_list[meshIndex].set_face_based(true);
                viewer.data_list[meshIndex].point_size = 2.0f;
                viewer.data_list[meshIndex].clear();
            }
            size_t meshIndex = viewer.mesh_index(o->getID());

            viewer.data_list[meshIndex].set_mesh(m_renderVs[i], m_renderFs[i]);
            viewer.data_list[meshIndex].compute_normals();

            Eigen::MatrixXd color;
            o->getColors(color);
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