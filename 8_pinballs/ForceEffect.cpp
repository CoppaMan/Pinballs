#include "ForceEffect.h"
#include <iostream>

ForceEffect::ForceEffect(PinballSim *sim, Eigen::Vector3d force, bool rel) : Effect(sim, 1.0), force(force), rel(rel) {

}

// Add the force to the other collider, either in relative or absolute coordinates
void ForceEffect::run(std::shared_ptr<RigidObject> other) {
    Eigen::Vector3d res = (rel ? parent->getRotation() : Eigen::Quaterniond::Identity()) * force;
    if(other->getType() == ObjType::DYNAMIC) {
        other->setLinearMomentum(other->getLinearMomentum() + res);
        std::cout << "Excert force: " << std::endl << res << std::endl;
    }
}