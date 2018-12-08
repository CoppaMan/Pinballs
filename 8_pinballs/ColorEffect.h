#pragma once

#include "Effect.h"

enum Fade {CONSTANT, LINEAR};

class ColorEffect : public Effect {
    public:
        ColorEffect(PinballSim *sim, Eigen::Vector3d targetCol, Fade fadeType, double duration);
        void run();
        void objectInit();
        void updateEffect();
    private:
        Fade fadeType;
        Eigen::MatrixXd originalColor;
        Eigen::MatrixXd targetColor;
        double duration;
        bool init = true;
        sf::Clock timer;
};