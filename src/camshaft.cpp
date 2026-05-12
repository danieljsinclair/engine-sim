#include "../include/camshaft.h"

#include "../include/crankshaft.h"
#include "../include/constants.h"
#include "../include/units.h"

#include <cmath>
#include <assert.h>

Camshaft::Camshaft() {
    m_crankshaft = nullptr;
    m_lobeAngles = nullptr;
    m_lobeProfile = nullptr;
    m_lobes = 0;
    m_advance = 0;
    m_baseRadius = 0;
}

Camshaft::~Camshaft() {
    assert(m_lobeAngles == nullptr);
}

void Camshaft::initialize(const Parameters &params) {
    m_lobeAngles = new real_t[params.lobes];
    memset(m_lobeAngles, 0, sizeof(real_t) * params.lobes);

    m_lobes = params.lobes;
    m_crankshaft = params.crankshaft;
    m_lobeProfile = params.lobeProfile;
    m_advance = params.advance;
    m_baseRadius = params.baseRadius;
}

void Camshaft::destroy() {
    delete[] m_lobeAngles;
    m_lobeAngles = nullptr;

    m_lobes = 0;
}

real_t Camshaft::valveLift(int lobe) const {
    return sampleLobe(getAngle() + m_lobeAngles[lobe]);
}

real_t Camshaft::sampleLobe(real_t theta) const {
    real_t clampedTheta = std::fmod(theta, (real_t)(2 * constants::pi));
    if (clampedTheta < 0) clampedTheta += (real_t)(2 * constants::pi);
    if (clampedTheta >= constants::pi) clampedTheta -= (real_t)(2 * constants::pi);

    return m_lobeProfile->sampleTriangle(clampedTheta);
}

real_t Camshaft::getAngle() const {
    const real_t angle =
        std::fmod((m_crankshaft->getAngle() + m_advance) * (real_t)0.5, (real_t)(2 * constants::pi));
    return (angle < 0)
        ?  angle + (real_t)(2 * constants::pi)
        :  angle;
}
