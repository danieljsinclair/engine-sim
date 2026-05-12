#include "../include/governor.h"

#include "../include/engine.h"
#include "../include/utilities.h"

Governor::Governor() {
    m_minSpeed = m_maxSpeed = 0;
    m_targetSpeed = 0;
    m_currentThrottle = 1.0f;
    m_velocity = 0.0f;
    m_minVelocity = m_maxVelocity = 0.0f;
    m_k_s = m_k_d = 0.0f;
    m_gamma = 1.0f;
}

Governor::~Governor() {
    /* void */
}

void Governor::initialize(const Parameters &params) {
    m_minSpeed = params.minSpeed;
    m_maxSpeed = params.maxSpeed;
    m_minVelocity = params.minVelocity;
    m_maxVelocity = params.maxVelocity;
    m_k_s = params.k_s;
    m_k_d = params.k_d;
    m_gamma = params.gamma;
}

void Governor::setSpeedControl(real_t s) {
    Throttle::setSpeedControl(s);

    m_targetSpeed = (1 - s) * m_minSpeed + s * m_maxSpeed;
}

void Governor::update(real_t dt, Engine *engine) {
    const real_t currentSpeed = engine->getSpeed();
    const real_t ds = m_targetSpeed * m_targetSpeed - currentSpeed * currentSpeed;

    m_velocity += (dt * -ds * m_k_s - m_velocity * dt * m_k_d);
    m_velocity = clamp(m_velocity, m_minVelocity, m_maxVelocity);

    if (std::abs(currentSpeed) < std::abs(0.5f * m_minSpeed)) {
        m_velocity = 0;
        m_currentThrottle = 1.0f;
    }

    m_currentThrottle += m_velocity * dt;
    m_currentThrottle = clamp(m_currentThrottle);

    engine->setThrottle(1 - std::pow(1 - m_currentThrottle, m_gamma));
}
