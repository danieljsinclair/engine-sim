#include "../include/crankshaft.h"

#include "../include/constants.h"

#include <cmath>
#include <assert.h>

Crankshaft::Crankshaft() {
    m_rodJournalAngles = nullptr;
    m_rodJournalCount = 0;
    m_throw = 0.0;
    m_m = 0.0;
    m_I = 0.0;
    m_flywheelMass = 0.0;
    m_p_x = m_p_y = 0.0;
    m_tdc = 0.0;
    m_frictionTorque = 0.0;
}

Crankshaft::~Crankshaft() {
    assert(m_rodJournalAngles == nullptr);
}

void Crankshaft::initialize(const Parameters &params) {
    m_m = params.mass;
    m_flywheelMass = params.flywheelMass;
    m_I = params.momentOfInertia;
    m_throw = params.crankThrow;
    m_rodJournalCount = params.rodJournals;
    m_rodJournalAngles = new real_t[m_rodJournalCount];
    m_p_x = params.pos_x;
    m_p_y = params.pos_y;
    m_tdc = params.tdc;
    m_frictionTorque = params.frictionTorque;
}

void Crankshaft::destroy() {
    if (m_rodJournalAngles != nullptr) delete[] m_rodJournalAngles;

    m_rodJournalAngles = nullptr;
}

void Crankshaft::getRodJournalPositionLocal(int i, real_t *x, real_t *y) {
    const real_t theta = m_rodJournalAngles[i];

    *x = std::cos(theta) * m_throw;
    *y = std::sin(theta) * m_throw;
}

void Crankshaft::getRodJournalPositionGlobal(int i, real_t *x, real_t *y) {
    real_t lx, ly;
    getRodJournalPositionLocal(i, &lx, &ly);

    *x = lx + (real_t)m_body.p_x;
    *y = ly + (real_t)m_body.p_y;
}

void Crankshaft::resetAngle() {
    m_body.theta = std::fmod((real_t)m_body.theta, (real_t)(4 * constants::pi));
}

void Crankshaft::setRodJournalAngle(int i, real_t angle) {
    assert(i < m_rodJournalCount && i >= 0);

    m_rodJournalAngles[i] = angle;
}

real_t Crankshaft::getAngle() const {
    return (real_t)m_body.theta - m_tdc;
}

real_t Crankshaft::getCycleAngle(real_t offset) {
    const real_t wrapped = std::fmod(-getAngle() + offset, (real_t)(4 * constants::pi));
    return (wrapped < 0)
        ? wrapped + (real_t)(4 * constants::pi)
        : wrapped;
}
