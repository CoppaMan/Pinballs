#include "RigidObject.h"

RigidObject::RigidObject(const std::string& mesh_file) {
    findAndLoadMesh(mesh_file);
    setMass(1.0);
    // inertia of cube
    setInertia(getMass() * 2.0 / 6.0 * Eigen::Matrix3d::Identity());
    reset();
}

void RigidObject::resetMembers() {
    setLinearMomentum(Eigen::Vector3d::Zero());
    setAngularMomentum(Eigen::Vector3d::Zero());
    resetForce();
    resetTorque();
}

void RigidObject::applyForceToCOM(const Eigen::Vector3d& f) { m_force += f; }

void RigidObject::applyForce(const Eigen::Vector3d& f,
                             const Eigen::Vector3d& p) {
    m_force += f;
    m_torque += (p - m_position).cross(f);
}

void RigidObject::applyTorque(const Eigen::Vector3d& t) { m_torque += t; }

#pragma region GettersAndSetters
void RigidObject::setMass(double m) {
    m_mass = m;
    m_massInv = 1.0 / m_mass;
}

void RigidObject::setInertia(const Eigen::Matrix3d& I) {
    m_inertia = I;
    m_inertiaInv = m_inertia.inverse();
}

void RigidObject::setLinearMomentum(const Eigen::Vector3d& p) { m_p = p; }

void RigidObject::setAngularMomentum(const Eigen::Vector3d& l) { m_l = l; }

void RigidObject::setLinearVelocity(const Eigen::Vector3d& v) {
    m_p = v * m_mass;
}

void RigidObject::setForce(const Eigen::Vector3d& f) { m_force = f; }

void RigidObject::setTorque(const Eigen::Vector3d& t) { m_torque = t; }

void RigidObject::resetForce() { m_torque.setZero(); };

void RigidObject::resetTorque() { m_torque.setZero(); };

double RigidObject::getMass() { return m_mass; }

double RigidObject::getMassInv() { return m_massInv; }

Eigen::Matrix3d RigidObject::getInertia() { return m_inertia; }

Eigen::Matrix3d RigidObject::getInertiaInv() { return m_inertiaInv; }

Eigen::Matrix3d RigidObject::getInertiaInvWorld() {
    return m_rot * m_inertiaInv * m_rot.transpose();
}

Eigen::Vector3d RigidObject::getLinearMomentum() { return m_p; }

Eigen::Vector3d RigidObject::getAngularMomentum() { return m_l; }

Eigen::Vector3d RigidObject::getLinearVelocity() { return m_p * m_massInv; }

Eigen::Vector3d RigidObject::getAngularVelocity() {
    return getInertiaInvWorld() * getAngularMomentum();
}

Eigen::Vector3d RigidObject::getForce() { return m_force; }

Eigen::Vector3d RigidObject::getTorque() { return m_torque; }

void RigidObject::getMesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F) {
    // get mesh after rotation and translation
    V = (m_mesh.V * m_scale * getRotationMatrix().transpose()).rowwise() +
        getPosition().transpose();
    F = m_mesh.F;
}
#pragma endregion GettersAndSetters