#include <igl/writeOFF.h>
#include <thread>
#include "Gui.h"
#include "Simulator.h"
#include "PinballSim.h"

/*
 * GUI for the spring simulation. This time we need additional paramters,
 * e.g. which integrator to use for the simulation and the force applied to the
 * canonball, and we also add some more visualizations (trajectories).
 */
class PinballGui : public Gui {
private:
    PinballSim *p_sim;
public:
    // simulation parameters


    PinballGui() {
        p_sim = new PinballSim();
        setSimulation(p_sim);

        start();
    }

    virtual void updateSimulationParameters() override {
        // change all parameters of the simulation to the values that are set in
        // the GUI
    }

    virtual void clearSimulation() override {
    }

    virtual void drawSimulationParameterMenu() override {
        ImGui::Text("Object");
    }
};

int main(int argc, char *argv[]) {
    // create a new instance of the GUI for the spring simulation
    new PinballGui();

    return 0;
}