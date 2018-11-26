#include "Effect.h"
#include "PinballSim.h"

class GravityEffect : public Effect {
    public:
        GravityEffect(PinballSim *sim, Eigen::Vector3d new_g);
        void run();
    private:
        Eigen::Vector3d gravity;
};