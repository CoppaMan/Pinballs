#pragma once

#include <igl/edges.h>
#include <memory>
#include <vector>
#include "SFML/System/Clock.hpp"
#include "RigidObject.h"

// forward declr
class PinballSim;
class RigidObject;

class Effect {
    public:
        Effect(PinballSim *sim, double cooldown);
        void apply();
        virtual void updateEffect();
        void addObject(std::shared_ptr<RigidObject> obj);
    protected:
        PinballSim *ps;
        std::shared_ptr<RigidObject> parent;
        virtual void run();
        virtual void objectInit();
    private:
        sf::Clock timer;
        double cooldown;
};