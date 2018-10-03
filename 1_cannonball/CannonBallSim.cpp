#include "CannonBallSim.h"

/////////////////////////////////////
//////// EX 1 - PROBLEM 1 ///////////
/////////////////////////////////////
bool CannonBallSim::advance() {
    // perform time integration with different integrators

	// use p_ball, m_dt, m_gravity
    switch (m_method) {
        case 0:
            // analytical solution
            break;

        case 1:
            // explicit euler
            break;

        case 2:
            // symplectic euler
            break;

        default:
            std::cerr << m_method << " is not a valid integrator method."
                        << std::endl;
    }

    // advance time
    m_time += m_dt;
    m_step++;

    // log
    if ((m_step % m_log_frequency) == 0) {
        m_trajectories.back().push_back(p_ball->getPosition());
    }

    return false;
}