#include "Paddle.h"

Paddle::Paddle(sf::Keyboard::Key k) : RigidObject("flipper.off", ObjType::DYNAMIC) {
    paddle_key = k;
}

void Paddle::toggle() {
    std::cout << "Right is supposed to be " << (int) sf::Keyboard::Key::Right << " It is " << (int) paddle_key << std::endl;
    Eigen::Matrix3d r = this->getRotationMatrix();
    double angle = std::atan2(-r(2,0), sqrt(r(2,1)*r(2,1) + r(2,2)*r(2,2)));
        //sf::Keyboard::Key key = sf::Keyboard::Key::Right;
    if (sf::Keyboard::isKeyPressed(paddle_key)) {
        if(angle < -1.0 or r(0,0) > 0) {
            this->setAngularMomentum(this->getRotation()*Eigen::Vector3d(0,-5,0));
        } else {
            this->setAngularMomentum(Eigen::Vector3d::Zero());
        }
    } else {
        if(angle < -0.8 or r(0,0) <= 0) {
            this->setAngularMomentum(this->getRotation()*Eigen::Vector3d(0,3,0));
        } else {
            this->setAngularMomentum(Eigen::Vector3d::Zero());
        }
    }
}