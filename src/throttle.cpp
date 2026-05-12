#include "../include/throttle.h"

Throttle::Throttle() {
    m_speedControl = 0.0f;
}

Throttle::~Throttle() {
    /* void */
}

void Throttle::setSpeedControl(real_t s) {
    m_speedControl = s;
}

void Throttle::update(real_t dt, Engine *engine) {
    /* void */
}
