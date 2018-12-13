#include "PinballSim.h"

bool PinballSim::advance() {

    // compute the collision detection
    float mt_old = m_dt;
    m_collisionDetection.computeCollisionDetection(m_dt, 1, 2, m_eps);

    if (p_ball->getPosition()(1) < -5 && ballOnTable) {
        balls--;
        ballOnTable = false;
    }

    if(!ballOnTable && balls > 0 && sf::Keyboard::isKeyPressed(sf::Keyboard::Down)) {
        p_ball->reset();
        p_ball->setPosition(Eigen::Vector3d(4.7, -2.5, 4.5));
        ballOnTable = true;
    }

    // add gravity only to ball
    p_ball->applyForceToCOM(m_gravity);

    p_paddle_r->toggle(); //Makes the paddles controlable
    p_paddle_l->toggle();

    //Score demo, press S key for more score
    if(sf::Keyboard::isKeyPressed(sf::Keyboard::S))score->addScore(1);

    for (auto o : m_objects) {
        for (auto e : o->getEffects()) e->updateEffect();
        //if(o->getType() != ObjType::DYNAMIC) continue;

        // integrate velocities
        o->setLinearMomentum(o->getLinearMomentum() + m_dt * o->getMassInv() * o->getForce());
        o->setAngularMomentum(o->getAngularMomentum() + m_dt * o->getMassInv()  * o->getTorque());
        o->resetForce();
        o->resetTorque();

        // angular velocity (matrix)
        Eigen::Vector3d w = o->getAngularVelocity();
        Eigen::Quaterniond wq;
        wq.w() = 0;
        wq.vec() = w;

        // integrate position and rotation
        o->setPosition(o->getPosition() + m_dt * o->getLinearVelocity());
        Eigen::Quaterniond q = o->getRotation();
        Eigen::Quaterniond dq = wq * q;
        Eigen::Quaterniond new_q;
        new_q.w() = q.w() + 0.5 * m_dt * dq.w();
        new_q.vec() = q.vec() + 0.5 * m_dt * dq.vec();
        o->setRotation(new_q.normalized());

    }


    // check if ball is under table and if yes move along the normal to get out of there
    Contact coni;
    if (m_collisionDetection.isTableCollision(p_ball, p_table->m_table_surface, coni)) {
        // if everything went smoothly we should not be here
        p_ball->setPosition(p_ball->getPosition() + coni.n * coni.penetration);
    }

    m_time += m_dt;
    m_step++;

    m_dt = mt_old;


    return false;
}

