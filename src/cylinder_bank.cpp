#include "../include/cylinder_bank.h"

#include "../include/constants.h"

#include <cmath>

CylinderBank::CylinderBank() {
    m_angle = 0.0;
    m_bore = 0.0;
    m_deckHeight = 0.0;
    m_cylinderCount = 0;
    m_displayDepth = 0.4;
    m_index = -1;

    m_dx = m_dy = 0;
    m_x = m_y = 0;
}

CylinderBank::~CylinderBank() {
    /* void */
}

void CylinderBank::initialize(const Parameters &params) {
    m_angle = params.angle;
    m_bore = params.bore;
    m_deckHeight = params.deckHeight;
    m_cylinderCount = params.cylinderCount;

    m_dx = std::cos(m_angle + (real_t)constants::pi / 2);
    m_dy = std::sin(m_angle + (real_t)constants::pi / 2);

    m_x = params.positionX;
    m_y = params.positionY;

    m_displayDepth = params.displayDepth;

    m_index = params.index;
}

void CylinderBank::destroy() {
    /* void */
}

void CylinderBank::getPositionAboveDeck(real_t h, real_t *x, real_t *y) const {
    *x = m_dx * (m_deckHeight + h) + m_x;
    *y = m_dy * (m_deckHeight + h) + m_y;
}

real_t CylinderBank::boreSurfaceArea() const {
    return (real_t)constants::pi * m_bore * m_bore / (real_t)4.0;
}
