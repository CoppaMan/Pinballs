#include "Paddle.h"

Paddle::Paddle(sf::Keyboard::Key k, Eigen::Vector3d p, bool look_left) : RigidObject("flipper.off", ObjType::DYNAMIC), paddle_key(k), pos(p), facing_left(look_left) {}

void Paddle::reset_paddle() { //Sets paddle back to original orientation
    setScale(0.2);
    setPosition(pos);

    Eigen::Matrix3d rotation_y;
    float theta = facing_left ? -M_PI/2.0 : M_PI/2.0; //rotate around UP (y-axis) counter-clock wise
    rotation_y << cos(theta), 0, sin(theta),
                0, 1, 0,
                -sin(theta), 0, cos(theta);

    Eigen::Matrix3d rotation_x;
    theta = facing_left ? -M_PI/6.0 : M_PI/6.0; //rotate around FRONT (z-axis) counter-clock wise
    rotation_x << std::cos(theta), -std::sin(theta), 0,
                std::sin(theta), std::cos(theta), 0,
                0, 0, 1;

    setRotation(rotation_y*(rotation_x));
}

void Paddle::toggle() { //Control paddle with key set in paddle_key
    Eigen::Matrix3d r = this->getRotationMatrix();
    double angle = std::atan2(-r(2,0), sqrt(r(2,1)*r(2,1) + r(2,2)*r(2,2)));

    if (sf::Keyboard::isKeyPressed(paddle_key)) {
        if(facing_left) {
            if(angle < -1.0 or r(0,0) > 0) {
                this->setAngularMomentum(this->getRotation()*Eigen::Vector3d(0, -5, 0));
            } else {
                this->setAngularMomentum(Eigen::Vector3d::Zero());
            }
        } else {
            if(r(0,0) > 0 or angle > 1.0) {
                this->setAngularMomentum(this->getRotation()*Eigen::Vector3d(0, 5, 0));
            } else {
                this->setAngularMomentum(Eigen::Vector3d::Zero());
            }
        }
    } else {
        if(facing_left) {
            if(angle < -0.8 or r(0,0) <= 0) {
                this->setAngularMomentum(this->getRotation()*Eigen::Vector3d(0, 3, 0));
            } else {
                this->setAngularMomentum(Eigen::Vector3d::Zero());
            }
        } else {
            if(r(0,0) <= 0 or angle > 0.8) {
                this->setAngularMomentum(this->getRotation()*Eigen::Vector3d(0, -3, 0));
            } else {
                this->setAngularMomentum(Eigen::Vector3d::Zero());
            }
        }
    }
}