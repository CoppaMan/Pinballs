#include "Effect.h"
#include <iostream>

Effect::Effect(PinballSim* sim, double cooldown) : ps(sim), cooldown(cooldown) {
}

void Effect::addObject(std::shared_ptr<RigidObject> obj) {
    parent = obj;
    objectInit();
}

void Effect::apply(std::shared_ptr<RigidObject> other) {
    if (timer.getElapsedTime().asSeconds() >= cooldown) {
        timer.restart();
        run(other);
    } else {
        std::cout << "Too early" << std::endl;
    }
}

void Effect::run(std::shared_ptr<RigidObject> other) {return;}
void Effect::updateEffect() {return;}
void Effect::objectInit() {return;}