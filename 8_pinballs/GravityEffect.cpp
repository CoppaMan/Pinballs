#include "GravityEffect.h"

GravityEffect::GravityEffect(PinballSim *sim, Eigen::Vector3d new_g) : Effect(sim, 10), gravity(new_g) {}

// Permanently change the gravity vector used for gravity
void GravityEffect::run(std::shared_ptr<RigidObject> other) {
    ps->m_gravity = gravity;
}