#include <RigidObject.h>
#include "SFML/Window/Keyboard.hpp"

class Paddle : public RigidObject {
    public:
        Paddle(sf::Keyboard::Key k);
        void toggle();
        sf::Keyboard::Key paddle_key;
};