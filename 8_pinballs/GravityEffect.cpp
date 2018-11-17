#include "GravityEffect.h"

GravityEffect::GravityEffect(PinballSim *sim, Eigen::Vector3d new_g) : gravity(new_g), Effect(sim) {}

void GravityEffect::apply() {
    ps->m_gravity = gravity;
}