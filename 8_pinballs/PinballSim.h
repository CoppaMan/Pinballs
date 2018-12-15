#pragma once

#include <igl/edges.h>
#include "Simulation.h"
#include "Table.h"
#include "Ball.h"
#include "CollisionDetection.h"
#include "Paddle.h"
#include "Score.h"
#include "Obstacle.h"
#include "ScoreEffect.h"
#include "ColorEffect.h"
#include "SoundEffect.h"
#include "ForceEffect.h"

//Import settings for blender:
//Forward is +X
//Up is +Y

//Green (Up) is +Y
//Blue (Forward) is +Z
//Red (Right) is +X

using namespace std;

class PinballSim : public Simulation {
public:
    PinballSim() : Simulation(), m_collisionDetection(m_objects) { init(); }

    float m_eps = 1.0;

    // CONFIGS
    float m_dt = 1e-2;
    Eigen::Vector3d m_gravity;
    int springStrength = 10;

    bool ballOnTable = false;
    int balls = 3;

    std::shared_ptr<Eigen::Vector3d> spring;

    // SCENE OBJECS
    std::shared_ptr<Table> p_table;
    std::shared_ptr<Ball> p_ball;
    std::shared_ptr<Paddle> p_paddle_r;
    std::shared_ptr<Paddle> p_paddle_l;
    std::shared_ptr<Obstacle> guard_l;
    std::shared_ptr<Obstacle> guard_r;
    std::shared_ptr<Score> score;
    std::shared_ptr<Obstacle> bumpL;
    std::shared_ptr<Obstacle> bumpT;
    std::shared_ptr<Obstacle> bumpB;
    std::shared_ptr<Obstacle> bumpR;
    std::shared_ptr<Obstacle> bounce;

    std::shared_ptr<ScoreEffect> add250points;
    std::shared_ptr<ScoreEffect> add1000points;
    std::shared_ptr<ColorEffect> blinkGreen, blinkBlue, blinkRed, blinkYellow;
    std::shared_ptr<ForceEffect> launch;
    std::shared_ptr<SoundEffect> dampSound;

    virtual void init() override {

        //Effects
        add250points = std::make_shared<ScoreEffect>(this, 250);
        add1000points = std::make_shared<ScoreEffect>(this, 1000);
        blinkBlue = std::make_shared<ColorEffect>(this, Eigen::Vector3d(0.0,0.0,1.0), Fade::LINEAR, 5.0);
        blinkGreen = std::make_shared<ColorEffect>(this, Eigen::Vector3d(0.0,1.0,0.0), Fade::LINEAR, 2.0);
        blinkRed = std::make_shared<ColorEffect>(this, Eigen::Vector3d(1.0,0.0,0.0), Fade::CONSTANT, 4.0);
        blinkYellow = std::make_shared<ColorEffect>(this, Eigen::Vector3d(1.0,1.0,0.0), Fade::CONSTANT, 3.0);
        dampSound = std::make_shared<SoundEffect>(this, "bump.ogg");

        m_objects.clear();
        p_ball = std::make_shared<Ball>();
        m_objects.emplace_back(p_ball);

        p_table = std::make_shared<Table>();
        p_table->emplaceInto(&m_objects);

        p_paddle_r = std::make_shared<Paddle>(p_table, sf::Keyboard::Key::Right, Eigen::Vector3d(1.3, 0, 6.2), true);
        m_objects.emplace_back(p_paddle_r); // Right paddle

        p_paddle_l = std::make_shared<Paddle>(p_table, sf::Keyboard::Key::Left, Eigen::Vector3d(-1.3, 0, 6.2), false);
        m_objects.emplace_back(p_paddle_l); // Left paddle

        bumpL = std::make_shared<Obstacle>(p_table, "bumper", 1, Eigen::Vector3d(-3,0,-4), Eigen::Vector3d(0.4, 0.4, 1), 0, false);
        bumpL->emplaceInto(&m_objects);
        bumpT = std::make_shared<Obstacle>(p_table, "bumper", 1, Eigen::Vector3d(-1.5,0,-5.5), Eigen::Vector3d(0.4, 1, 0.4), 0, false);
        bumpT->emplaceInto(&m_objects);
        bumpB = std::make_shared<Obstacle>(p_table, "bumper", 1, Eigen::Vector3d(-1.5,0,-2.5), Eigen::Vector3d(1, 0.4, 0.4), 0, false);
        bumpB->emplaceInto(&m_objects);
        bumpR = std::make_shared<Obstacle>(p_table, "bumper", 1, Eigen::Vector3d(0,0,-4), Eigen::Vector3d(1, 1, 0.4), 0, false);
        bumpR->emplaceInto(&m_objects);

        guard_l = std::make_shared<Obstacle>(p_table, "guard", 4, Eigen::Vector3d(-3,0,5), Eigen::Vector3d(1, 1, 1), 0, false);
        guard_l->emplaceInto(&m_objects);

        guard_r = std::make_shared<Obstacle>(p_table, "guard", 4, Eigen::Vector3d(3,0,5), Eigen::Vector3d(1, 1, 1), 0, true);
        guard_r->emplaceInto(&m_objects);

        bounce = std::make_shared<Obstacle>(p_table, "bounce", 1, Eigen::Vector3d(4.7,0,5.5), Eigen::Vector3d(1, 0.4, 0.4), 0, false);
        bounce->emplaceInto(&m_objects);

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

        p_table->resetTable();

        p_ball->setScale(0.008);
        p_ball->setPosition(Eigen::Vector3d(0, -10, 0));
        p_ball->setMass(1);
        Eigen::MatrixXd color(1, 3);
        color << 0.0, 204.0 / 255.0, 102.0 / 255.0;
        p_ball->setColors(color);

        p_paddle_r->reset_paddle(); //resets the paddle state
        p_paddle_l->reset_paddle();

        guard_l->resetObstacle();
        guard_r->resetObstacle();

        bumpL->resetObstacle();
        bumpL->addEffect(add1000points);
        bumpL->addEffect(blinkBlue);
        bumpL->addEffect(dampSound);

        bumpT->resetObstacle();
        bumpT->addEffect(add1000points);
        bumpT->addEffect(blinkGreen);
        bumpT->addEffect(dampSound);

        bumpR->resetObstacle();
        bumpR->addEffect(add1000points);
        bumpR->addEffect(blinkYellow);
        bumpR->addEffect(dampSound);

        bumpB->resetObstacle();
        bumpB->addEffect(add1000points);
        bumpB->addEffect(blinkRed);
        bumpB->addEffect(dampSound);

        bounce->resetObstacle();
        Eigen::Vector3d spring(0, 0, -springStrength);
        launch = std::make_shared<ForceEffect>(this, spring, true);
        bounce->addEffect(launch);

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