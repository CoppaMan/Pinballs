#pragma once

#include <igl/edges.h>
#include "SFML/System/Clock.hpp"

// forward declr
class PinballSim;

class Effect {
    public:
        Effect(PinballSim *sim, double cooldown);
        void apply();
    protected:
        PinballSim *ps;
        virtual void run();
    private:
        sf::Clock timer;
        double cooldown;
};