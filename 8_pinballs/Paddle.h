#include <RigidObject.h>
#include <SFML/Window/Keyboard.hpp>
#include <math.h>

class Paddle : public RigidObject {
public:
    Paddle(sf::Keyboard::Key k) : RigidObject("flipper.off", ObjType::DYNAMIC) {key = k;}

    void toggle() {
        Eigen::Matrix3d r = getRotationMatrix();
        double angle = std::atan2(-r(2,0), sqrt(r(2,1)*r(2,1) + r(2,2)*r(2,2)));
        std::cout << key << std::endl;
        if (sf::Keyboard::isKeyPressed(key)) {
            if(angle < -1.0 or r(0,0) > 0) {
                std::cout << "UP" << std::endl;
                setAngularMomentum(getRotation()*Eigen::Vector3d(0,-5,0));
            } else {
                setAngularMomentum(Eigen::Vector3d::Zero());
            }
        } else {
            if(angle < -0.8 or r(0,0) <= 0) {
                std::cout << "DOWN" << std::endl;
                setAngularMomentum(getRotation()*Eigen::Vector3d(0,3,0));
            } else {
                setAngularMomentum(Eigen::Vector3d::Zero());
            }
        }
    }
private:
    sf::Keyboard::Key key;
};