#include <Effect.h>

class GravityEffect : public Effect {
    public:
        GravityEffect(PinballSim sim, Eigen::Vector3d new_g);
        void apply() override;
    private:
        Eigen::Vector3d gravity;
};