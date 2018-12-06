#include "Paddle.h"
#include <iostream>

Paddle::Paddle(std::shared_ptr<Table> table, sf::Keyboard::Key k, Eigen::Vector3d p, bool look_left) : RigidObject("flipper.off", ObjType::ROTATION_ONLY), parent(table), paddle_key(k), facing_left(look_left) {
    pos = table->getRotation()*p;

    Eigen::Matrix3d flip;
    double phi = M_PI; 
    flip << cos(phi), sin(phi), 0,
            -sin(phi), cos(phi), 0,
            0, 0, 1;

    loc_rot = Eigen::Matrix3d::Identity();
    abs_rot = look_left ? parent->getRotation() * flip : parent->getRotation();
}

void Paddle::reset_paddle() { //Sets paddle back to original orientation
    setScale(0.2);
    setPosition(pos);
    setRotation(abs_rot);
}

void Paddle::toggle() { //Control paddle with key set in paddle_key
    Eigen::Vector3d axis = 3*abs_rot*Eigen::Vector3d(0, 1, 0);

    Eigen::Matrix3d flip;
    double phi = M_PI; 
    flip << cos(phi), sin(phi), 0,
            -sin(phi), cos(phi), 0,
            0, 0, 1;

    loc_rot = (parent->getRotation().transpose() * getRotationMatrix());
    if(facing_left) {
        loc_rot = flip.transpose() * loc_rot;
    }
    //std::cout << loc_rot << std::endl;

    if (sf::Keyboard::isKeyPressed(paddle_key)) {
        if(loc_rot(0,0) > 0.8 || loc_rot(0,2) < 0) {
            setAngularMomentum(axis);
        } else {
            setAngularMomentum(Eigen::Vector3d::Zero());
        }
    } else {
        if(loc_rot(0,0) > 0.88 || loc_rot(0,2) > 0) {
            setAngularMomentum(-axis);
        } else {
            setAngularMomentum(Eigen::Vector3d::Zero());
        }
    }
}