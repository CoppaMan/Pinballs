#include "SoundEffect.h"
#include <iostream>

SoundEffect::SoundEffect(PinballSim *sim, std::string name) : Effect(sim, 0.2) {
    std::stringstream stream;
    stream << "../../sound/" << name;
    buffer.loadFromFile(stream.str());
}

void SoundEffect::run(std::shared_ptr<RigidObject> other) {
    sf::Sound sound;
    sound.setBuffer(buffer);
    sound.setVolume(100);
    sound.play();
}