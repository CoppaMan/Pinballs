#include "ForceEffect.h"
#include <iostream>

ForceEffect::ForceEffect(PinballSim *sim, Eigen::Vector3d force, bool rel) : Effect(sim, 1.0), force(force), rel(rel) {

}

void ForceEffect::run(std::shared_ptr<RigidObject> other) {
    std::cout << "Excert force" << std::endl;
    if(other->getType() == ObjType::DYNAMIC)
        other->setLinearVelocity(other->getLinearVelocity() + (rel ? parent->getRotation() : Eigen::Quaterniond::Identity()) * force);
}