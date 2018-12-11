#pragma once

#include "Effect.h"

class ScoreEffect : public Effect {
    public:
        ScoreEffect(PinballSim *sim, long new_v);
        void run(std::shared_ptr<RigidObject> other);
    private:
        long value;
};