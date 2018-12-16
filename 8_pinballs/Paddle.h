#pragma once

#include "RigidObject.h"
#include "Table.h"
#include "SFML/Window/Keyboard.hpp"
#include "SFML/Audio.hpp"

class Paddle : public RigidObject {
    public:
        Paddle(std::shared_ptr<Table> table, sf::Keyboard::Key k, Eigen::Vector3d p, bool look_left);
        void toggle();
        void reset_paddle();
    private:
        std::shared_ptr<Table> parent;
        sf::Keyboard::Key paddle_key;
        Eigen::Vector3d pos;
        Eigen::Matrix3d loc_rot, abs_rot;
        bool facing_left = true;
        bool active = false;
        sf::SoundBuffer bufferActive, bufferInactive;
        sf::Sound soundActive, soundInactive;
};