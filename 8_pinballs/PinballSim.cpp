#include "PinballSim.h"

bool PinballSim::advance() {
    // advance m_time
    m_time += m_dt;
    m_step++;

    return false;
}