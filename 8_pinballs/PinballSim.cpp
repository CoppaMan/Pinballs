#include "PinballSim.h"

bool PinballSim::advance() {

    // compute the collision detection
    //m_collisionDetection.computeCollisionDetection(1, 1, m_eps);

    // add gravity only to ball
    p_ball->applyForceToCOM(m_gravity);

    p_paddle_r->printDebug();

    //p_paddle_r->toggle();

    Eigen::Matrix3d r = p_paddle_r->getRotationMatrix();
        double angle = std::atan2(-r(2,0), sqrt(r(2,1)*r(2,1) + r(2,2)*r(2,2)));
        sf::Keyboard::Key key = sf::Keyboard::Key::Right;
        if (sf::Keyboard::isKeyPressed(key)) {
            if(angle < -1.0 or r(0,0) > 0) {
                std::cout << "UP" << std::endl;
                p_paddle_r->setAngularMomentum(p_paddle_r->getRotation()*Eigen::Vector3d(0,-5,0));
            } else {
                p_paddle_r->setAngularMomentum(Eigen::Vector3d::Zero());
            }
        } else {
            if(angle < -0.8 or r(0,0) <= 0) {
                std::cout << "DOWN" << std::endl;
                p_paddle_r->setAngularMomentum(p_paddle_r->getRotation()*Eigen::Vector3d(0,3,0));
            } else {
                p_paddle_r->setAngularMomentum(Eigen::Vector3d::Zero());
            }
        }

    for (auto &o : m_objects) {

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

