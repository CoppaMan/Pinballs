#include <igl/edges.h>
#include "Simulation.h"

using namespace std;

class PinballSim : public Simulation {
public:
    PinballSim() : Simulation() { init(); }

    RigidObject* p_table;
    RigidObject* p_ball;

    virtual void init() override {

        m_objects.clear();
        m_objects.push_back(RigidObject("table.obj"));
        p_table = &m_objects.back();
        m_objects.push_back(RigidObject("sphere.off"));
        p_ball = &m_objects.back();

        reset();
    }

    virtual void resetMembers() override {
        for (auto &o : m_objects) {
            o.reset();
        }

        p_table->setScale(1);

        p_ball->setScale(0.002);
        p_ball->setPosition(Eigen::Vector3d(0, -0.2, 0));

        Eigen::MatrixXd color(1, 3);
        color << 0.0, 204.0 / 255.0, 102.0 / 255.0;
        p_ball->setColors(color);


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

    virtual bool advance() override;

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

};