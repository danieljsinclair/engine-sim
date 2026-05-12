#include "../include/connecting_rod.h"

#include <cmath>

ConnectingRod::ConnectingRod() {
    m_centerOfMass = 0.0;
    m_length = 0.0;
    m_m = 0.0;
    m_I = 0.0;
    m_journal = 0;
    m_master = nullptr;
    m_crankshaft = nullptr;
    m_piston = nullptr;
    m_slaveThrow = 0;

    m_rodJournalAngles = nullptr;
    m_rodJournalCount = 0;
}

ConnectingRod::~ConnectingRod() {
    /* void */
}

void ConnectingRod::initialize(const Parameters &params) {
    m_centerOfMass = params.centerOfMass;
    m_length = params.length;
    m_m = params.mass;
    m_I = params.momentOfInertia;
    m_journal = params.journal;
    m_crankshaft = params.crankshaft;
    m_piston = params.piston;

    m_rodJournalAngles = new real_t[params.rodJournals];
    m_rodJournalCount = params.rodJournals;
    m_slaveThrow = params.slaveThrow;
    m_master = params.master;
}

real_t ConnectingRod::getBigEndLocal() const {
    return -(m_length / 2) + m_centerOfMass;
}

real_t ConnectingRod::getLittleEndLocal() const {
    return (m_length / 2) - m_centerOfMass;
}

void ConnectingRod::setRodJournalAngle(int i, real_t angle) {
    m_rodJournalAngles[i] = angle;
}

void ConnectingRod::getRodJournalPositionLocal(int i, real_t *x, real_t *y) {
    const real_t journalAngle = getRodJournalAngle(i);
    const real_t journal_x_local = std::cos(journalAngle) * m_slaveThrow;
    const real_t journal_y_local = std::sin(journalAngle) * m_slaveThrow;

    *x = journal_x_local;
    *y = journal_y_local + getBigEndLocal();
}

void ConnectingRod::getRodJournalPositionGlobal(int i, real_t *x, real_t *y) {
    real_t lx, ly;
    getRodJournalPositionLocal(i, &lx, &ly);

    const real_t angle = m_body.theta;
    const real_t dx = std::cos(angle);
    const real_t dy = std::sin(angle);

    *x = (dx * lx - dy * ly) + m_body.p_x;
    *y = (dy * lx + dx * ly) + m_body.p_y;
}

int ConnectingRod::getLayer() const {
    if (m_master != nullptr) {
        return m_master->getLayer();
    }
    else {
        return getJournal();
    }
}
