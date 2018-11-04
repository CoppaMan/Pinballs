#include <igl/edges.h>
#include "Simulation.h"

using namespace std;

class PinballSim : public Simulation {
public:
    PinballSim() : Simulation() { init(); }

    RigidObject* p_table;

    virtual void init() override {
        std::string path = "table.obj";
        p_table = new RigidObject(path);
        m_objects.clear();
        m_objects.push_back(*p_table);

        reset();
    }

    virtual void resetMembers() override {
        p_table->reset();
        p_table->setScale(1);

    }

    virtual void updateRenderGeometry() override {
        p_table->getMesh(m_renderV, m_renderF);
    }

    virtual bool advance() override;

    virtual void renderRenderGeometry(igl::opengl::glfw::Viewer &viewer) override {
        viewer.data().set_mesh(m_renderV, m_renderF);
    }

#pragma region SettersAndGetters

#pragma endregion SettersAndGetters

private:
    Eigen::MatrixXd m_renderV;  // vertex positions for rendering
    Eigen::MatrixXi m_renderF;  // face indices for rendering
};