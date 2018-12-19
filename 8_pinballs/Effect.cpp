#include "Effect.h"
#include <iostream>

Effect::Effect(PinballSim* sim, double cooldown) : ps(sim), cooldown(cooldown) {
}

// Makes the effect know who is attaching it to it
void Effect::addObject(std::shared_ptr<RigidObject> obj) {
    parent = obj;
    objectInit();
}

// Wrapper for run, making sure to only trigger once during cooldown
void Effect::apply(std::shared_ptr<RigidObject> other) {
    if (timer.getElapsedTime().asSeconds() >= cooldown) {
        timer.restart();
        run(other);
    } else {
        std::cout << "Too early" << std::endl;
    }
}

// All three methods below can be overwritten if necessary, providing an init, update and run entry point 
void Effect::run(std::shared_ptr<RigidObject> other) {return;}
void Effect::updateEffect() {return;}
void Effect::objectInit() {return;}