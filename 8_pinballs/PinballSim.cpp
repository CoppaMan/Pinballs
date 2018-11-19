#include "PinballSim.h"

bool PinballSim::advance() {

    // compute the collision detection
    m_collisionDetection.computeCollisionDetection(1, 1, m_eps);

    // add gravity only to ball
    p_ball->applyForceToCOM(m_gravity);

    for (auto &o : m_objects) {
        //Apply rotation to flippers
        if(o.getType() == ObjType::ROTATION_ONLY) {
            if(/*corresponding key presses*/0) {
                o.setAngularMomentum(4*Eigen::Vector3d::UnitZ());
            } else {
                o.setAngularMomentum(-4*Eigen::Vector3d::UnitZ());
            }
        }

        // integrate velocities
        o.setLinearMomentum(o.getLinearMomentum() + m_dt * o.getMassInv() * o.getForce());
        o.setAngularMomentum(o.getAngularMomentum() + m_dt * o.getMassInv()  * o.getTorque());
        o.resetForce();
        o.resetTorque();

        // angular velocity (matrix)
        Eigen::Vector3d w = o.getAngularVelocity();
        Eigen::Quaterniond wq;
        wq.w() = 0;
        wq.vec() = w;

        // integrate position and rotation
        o.setPosition(o.getPosition() + m_dt * o.getLinearVelocity());
        Eigen::Quaterniond q = o.getRotation();
        Eigen::Quaterniond dq = wq * q;
        Eigen::Quaterniond new_q;
        new_q.w() = q.w() + 0.5 * m_dt * dq.w();
        new_q.vec() = q.vec() + 0.5 * m_dt * dq.vec();
        o.setRotation(new_q.normalized());
    }


    m_time += m_dt;
    m_step++;

    return false;
}

