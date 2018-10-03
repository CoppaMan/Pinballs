#include "GyroSim.h"

/////////////////////////////////////
//////// EX 2 - PROBLEM 1 ///////////
/////////////////////////////////////
bool GyroSim::advance() {
	// update orientation
	switch (m_method) {
	case 0: {
		// semi-implicit
		break;
	}
	case 1: {
		// solve gyroscopic
		break;
	}
	}

	// advance time
	m_time += m_dt;
	m_step++;

	return false;
}