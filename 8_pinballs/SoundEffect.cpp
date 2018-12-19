#include "SoundEffect.h"
#include <iostream>

// Preparing the sound buffers, they can be used by multiple objects
SoundEffect::SoundEffect(PinballSim *sim, std::string name) : Effect(sim, 0.2) {
    std::stringstream stream;
    stream << "../../sound/" << name;
    buffer.loadFromFile(stream.str());
    sound.setBuffer(buffer);
    sound.setVolume(100);
}

// Add some pitch shift to the sound
void SoundEffect::run(std::shared_ptr<RigidObject> other) {
    float shift = (rand() % 10) - 5;
    sound.setPitch(shift + sound.getPitch());
    sound.play();
}