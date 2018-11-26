#include "GravityEffect.h"

GravityEffect::GravityEffect(PinballSim *sim, Eigen::Vector3d new_g) : Effect(sim, 10), gravity(new_g) {}

void GravityEffect::run() {
    ps->m_gravity = gravity;
}