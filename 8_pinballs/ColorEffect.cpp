#pragma once

#include "ColorEffect.h"
#include <iostream>

ColorEffect::ColorEffect(PinballSim *sim, Eigen::Vector3d targetCol, Fade fadeType, double duration) :
Effect(sim, 0.5), fadeType(fadeType), duration(duration) {
    targetColor = Eigen::MatrixXd(1,3);
    targetColor << targetCol(0), targetCol(1), targetCol(2);
}

void ColorEffect::run(std::shared_ptr<RigidObject> other) {
    if(init) {
        timer.restart();
        init = false;
        std::cout << "Color change from " << originalColor << " to " << targetColor << "!" << std::endl;
    }
}

void ColorEffect::updateEffect() {
    if(!init) {
        if(timer.getElapsedTime().asSeconds() < duration) { // Still in transition
            double frac = timer.getElapsedTime().asSeconds()/duration;
            Eigen::MatrixXd mix(1,3);

            switch(fadeType) {
                case Fade::CONSTANT:
                    mix = targetColor;
                    break;
                case Fade::LINEAR:
                    mix = (1-frac)*targetColor + frac*originalColor;
                    break;
            }

            const Eigen::MatrixXd current = mix;
            parent->setColors(current);
        } else { // Change back to original color
            const Eigen::MatrixXd current = originalColor;
            parent->setColors(current);
            init = true;
        }
    }
}

void ColorEffect::objectInit() {
    parent->getColors(originalColor);
}