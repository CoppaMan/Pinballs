#pragma once

#include "SFML/Audio.hpp"
#include "Effect.h"
#include <string>

class SoundEffect : public Effect {
    public:
        SoundEffect(PinballSim *sim, std::string name);
        void run(std::shared_ptr<RigidObject> other);
    private:
        std::string path;
        sf::SoundBuffer buffer;
        sf::Sound sound;
};