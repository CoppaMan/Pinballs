#include "SpinSim.h"

/////////////////////////////////////
//////// EX 2 - PROBLEM 1 ///////////
/////////////////////////////////////
bool SpinSim::advance() {
	// update orientation
	switch (m_method) {
	case 0: {
		// matrix-based angular velocity
		break;
	}
	case 1: {
		// quaternion-based angular velocity
		break;
	}
	}

	// advance time
	m_time += m_dt;
	m_step++;

	return false;
}