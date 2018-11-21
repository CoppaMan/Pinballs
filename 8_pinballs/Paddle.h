#include <RigidObject.h>
#include "SFML/Window/Keyboard.hpp"

class Paddle : public RigidObject {
    public:
        Paddle(sf::Keyboard::Key k, Eigen::Vector3d p, bool look_left);
        void toggle();
        void reset_paddle();
    private:
        sf::Keyboard::Key paddle_key;
        Eigen::Vector3d pos;
        bool facing_left = true;
};