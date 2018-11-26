#include "Effect.h"
#include <iostream>

Effect::Effect(PinballSim* sim, double cooldown) : ps(sim), cooldown(cooldown) {
}

void Effect::apply() {
    if (timer.getElapsedTime().asSeconds() >= cooldown) {
        timer.restart();
        run();
    } else {
        std::cout << "Too early" << std::endl;
    }
}

void Effect::run() {return;}