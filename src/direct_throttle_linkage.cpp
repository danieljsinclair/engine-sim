#include "../include/direct_throttle_linkage.h"

#include "../include/engine.h"

#include <cmath>

DirectThrottleLinkage::DirectThrottleLinkage() {
    m_gamma = 1.0f;
    m_throttlePosition = 1.0f;
}

DirectThrottleLinkage::~DirectThrottleLinkage() {
    /* void */
}

void DirectThrottleLinkage::initialize(const Parameters &params) {
    m_gamma = params.gamma;
}

void DirectThrottleLinkage::setSpeedControl(real_t s) {
    Throttle::setSpeedControl(s);
    m_throttlePosition = 1 - std::pow(s, m_gamma);
}

void DirectThrottleLinkage::update(real_t dt, Engine *engine) {
    Throttle::update(dt, engine);
    engine->setThrottle(m_throttlePosition);
}
