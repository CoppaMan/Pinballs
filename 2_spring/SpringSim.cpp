#include "SpringSim.h"

/////////////////////////////////////
///////////// PROBLEM 1 /////////////
/////////////////////////////////////
bool SpringSim::advance() {
    // perform time integration with different integrators

	// use p_cube, m_spring, m_dt, m_gravity

	// useful functions: normalized(), norm()
    Eigen::Vector3d spring_dir =
        (m_spring.start - m_spring.end).normalized();
	float spring_norm = (m_spring.start - m_spring.end).norm();

	// note that it is required to update both m_sptring.end and p_cube's position
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

        case 3:
            // explicit midpoint            
            break;

        case 4:
            // implicit euler            
            break;
        default:
            std::cerr << m_method << " is not a valid integrator method."
                        << std::endl;
    }

    // advance m_time
    m_time += m_dt;
    m_step++;

    // log
    if ((m_step % m_log_frequency) == 0) {
        m_trajectories.back().push_back(p_cube->getPosition());
    }

    return false;
}