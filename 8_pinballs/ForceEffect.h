#pragma once

#include "Effect.h"

class ForceEffect : public Effect {
    public:
        ForceEffect(PinballSim *sim, Eigen::Vector3d force, bool rel);
        void run(std::shared_ptr<RigidObject> other);
    private:
        Eigen::Vector3d force;
        bool rel;
};