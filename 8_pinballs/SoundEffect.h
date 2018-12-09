#pragma once

#include "SFML/Audio.hpp"
#include "Effect.h"
#include <string>

class SoundEffect : public Effect {
    public:
        SoundEffect(PinballSim *sim, std::string name);
        void run();
    private:
        std::string path;
        sf::SoundBuffer buffer;
};