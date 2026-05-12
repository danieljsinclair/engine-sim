#ifndef ATG_ENGINE_SIM_GAS_SYSTEM_H
#define ATG_ENGINE_SIM_GAS_SYSTEM_H

#include "constants.h"
#include "units.h"

#include <cfloat>
#include <cmath>

class GasSystem {
    public:
        struct Mix {
            real_t p_fuel = 0.0;
            real_t p_inert = 1.0;
            real_t p_o2 = 0.0;
        };

        struct State {
            real_t n_mol = 0.0;
            real_t E_k = 0.0;
            real_t V = 0.0;
            real_t momentum[2] = { 0.0, 0.0 };

            Mix mix;
        };

        struct FlowParameters {
            real_t k_flow;
            real_t dt;
            real_t direction_x, direction_y;
            real_t crossSectionArea_0, crossSectionArea_1;
            GasSystem *system_0, *system_1;
        };

    public:
        GasSystem() { /* void */ }
        ~GasSystem() { /* void */ }

        void setGeometry(real_t width, real_t height, real_t dx, real_t dy);
        void initialize(real_t P, real_t V, real_t T, const Mix &mix, int degreesOfFreedom = 5);
        void initialize(real_t P, real_t V, real_t T, int degreesOfFreedom = 5);
        void reset(real_t P, real_t T, const Mix &mix);
        void reset(real_t P, real_t T);

        void setVolume(real_t V);
        void setN(real_t n);

        void changeVolume(real_t dV);
        void changePressure(real_t pressure);
        void changeTemperature(real_t dT);
        void changeTemperature(real_t dT, real_t n);
        void changeEnergy(real_t dE);
        void changeMix(const Mix &mix);
        void injectFuel(real_t n);

        real_t react(real_t n, const Mix &mix);
        static real_t flowConstant(real_t flowRate, real_t P, real_t pressureDrop, real_t T, real_t hcr);
        static real_t k_28inH2O(real_t flowRateScfm);
        static real_t k_carb(real_t flowRateScfm);
        static real_t flowRate(
            real_t k_flow,
            real_t P0,
            real_t P1,
            real_t T0,
            real_t T1,
            real_t hcr,
            real_t chokedFlowLimit,
            real_t chokedFlowRateCached);
        real_t loseN(real_t dn, real_t E_k_per_mol);
        real_t gainN(real_t dn, real_t E_k_per_mol, const Mix &mix);
        real_t gainN(real_t dn, real_t E_k_per_mol);
        void dissipateExcessVelocity();

        void updateVelocity(real_t dt, real_t beta = 1.0);
        void dissipateVelocity(real_t dt, real_t timeConstant);

        static real_t flow(const FlowParameters &params);
        real_t flow(real_t k_flow, real_t dt, real_t P_env, real_t T_env, const Mix &mix);
        real_t flow(real_t k_flow, real_t dt, real_t P_env, real_t T_env);

        real_t pressureEquilibriumMaxFlow(const GasSystem *b) const;
        real_t pressureEquilibriumMaxFlow(real_t P_env, real_t T_env) const;

        inline static constexpr real_t kineticEnergyPerMol(real_t T, int degreesOfFreedom);
        inline static constexpr real_t heatCapacityRatio(int degreesOfFreedom);
        inline static real_t chokedFlowLimit(int degreesOfFreedom);
        inline static real_t chokedFlowRate(int degreesOfFreedom);

        inline real_t approximateDensity() const;
        inline int degreesOfFreedom() const { return m_degreesOfFreedom; }
        inline real_t n() const;
        inline real_t n(real_t V) const;
        inline real_t kineticEnergy() const;
        inline real_t kineticEnergy(real_t n) const;
        inline real_t kineticEnergyPerMol() const { return kineticEnergy(1.0); }
        inline real_t totalEnergy() const;
        inline real_t bulkKineticEnergy() const;
        inline real_t c() const;
        inline real_t dynamicPressure(real_t dx, real_t dy) const;
        inline real_t mass() const;
        inline real_t pressure() const;
        inline real_t temperature() const;
        inline real_t velocity_x() const;
        inline real_t velocity_y() const;
        inline real_t volume() const;
        inline real_t volume(real_t n) const;
        inline real_t n_fuel() const;
        inline real_t n_inert() const;
        inline real_t n_o2() const;
        inline real_t heatCapacityRatio() const;
        inline Mix mix() const { return m_state.mix; }

    protected:
        State m_state;

        int m_degreesOfFreedom = 5;

        real_t m_chokedFlowLimit = 0;
        real_t m_chokedFlowFactorCached = 0;

        real_t m_width = 0.0;
        real_t m_height = 0.0;
        real_t m_dx = 0.0;
        real_t m_dy = 0.0;
};

inline constexpr real_t GasSystem::kineticEnergyPerMol(real_t T, int degreesOfFreedom) {
    return 0.5 * T * constants::R * degreesOfFreedom;
}

inline constexpr real_t GasSystem::heatCapacityRatio(int degreesOfFreedom) {
    return 1.0 + (2.0 / degreesOfFreedom);
}

inline real_t GasSystem::chokedFlowLimit(int degreesOfFreedom) {
    const real_t hcr = heatCapacityRatio(degreesOfFreedom);
    return std::pow((2.0 / (hcr + 1)), hcr / (hcr - 1));
}

inline real_t GasSystem::chokedFlowRate(int degreesOfFreedom) {
    const real_t hcr = heatCapacityRatio(degreesOfFreedom);
    real_t flowRate =
        std::sqrt(hcr) * std::pow(2 / (hcr + 1), (hcr + 1) / (2 * (hcr - 1)));

    return flowRate;
}

inline real_t GasSystem::approximateDensity() const {
    return (units::AirMolecularMass * n()) / volume();
}

inline real_t GasSystem::n() const {
    return m_state.n_mol;
}

inline real_t GasSystem::n(real_t V) const {
    return (V / volume()) * n();
}

inline real_t GasSystem::kineticEnergy() const {
    return m_state.E_k;
}

inline real_t GasSystem::kineticEnergy(real_t n) const {
    return (kineticEnergy() / this->n()) * n;
}

inline real_t GasSystem::c() const {
    if (n() == 0 || kineticEnergy() == 0) return 0;

    const real_t hcr = heatCapacityRatio();
    const real_t staticPressure = pressure();
    const real_t density = approximateDensity();
    const real_t c = std::sqrt(staticPressure * hcr / density);

    return c;
}

inline real_t GasSystem::totalEnergy() const {
    if (n() == 0) return 0;

    const real_t invMass = 1 / mass();
    const real_t v_x = m_state.momentum[0] * invMass;
    const real_t v_y = m_state.momentum[1] * invMass;
    const real_t v_squared = v_x * v_x + v_y * v_y;

    return kineticEnergy() + 0.5 * mass() * v_squared;
}

inline real_t GasSystem::bulkKineticEnergy() const {
    const real_t m = mass();
    if (m == 0) return 0;

    const real_t v_x = m_state.momentum[0] / m;
    const real_t v_y = m_state.momentum[1] / m;
    const real_t v_squared = v_x * v_x + v_y * v_y;
    return 0.5 * m * v_squared;
}

inline real_t GasSystem::dynamicPressure(real_t dx, real_t dy) const {
    if (n() == 0 || kineticEnergy() == 0) return 0;

    const real_t inverseMass = 1 / this->mass();
    const real_t v = inverseMass * (dx * m_state.momentum[0] + dy * m_state.momentum[1]);

    if (v <= 0) {
        return 0;
    }

    const real_t hcr = heatCapacityRatio();
    const real_t staticPressure = pressure();
    const real_t density = approximateDensity();
    const real_t c_squared = staticPressure * hcr / density;
    const real_t machNumber_squared = v * v / c_squared;

    // Below is equivalent to:
    // staticPressure * pow(1 + ((hcr - 1) / 2) * machNumber * machNumber, hcr / (hcr - 1)) - 1)

    const real_t x = 1 + ((hcr - 1) / 2) * machNumber_squared;
    real_t x_d;
    switch (m_degreesOfFreedom) {
    case 3:
        x_d = x * x * x * x * x;
        break;
    case 5:
    {
        const real_t x_2 = x * x;
        const real_t x_3 = x_2 * x;
        x_d = x_3 * x_3 * x;
        break;
    }
    default:
        x_d = x;
    }

    return staticPressure * (std::sqrt(x_d) - 1);
}

inline real_t GasSystem::mass() const {
    return units::AirMolecularMass * n();
}

inline real_t GasSystem::pressure() const {
    const real_t volume = this->volume();
    return (volume != 0)
        ? kineticEnergy() / (0.5 * m_degreesOfFreedom * volume)
        : 0;
}

inline real_t GasSystem::temperature() const {
    if (n() == 0) return 0;
    else return kineticEnergy() / (0.5 * m_degreesOfFreedom * n() * constants::R);
}

inline real_t GasSystem::velocity_x() const {
    if (n() == 0) return 0;
    else return m_state.momentum[0] / mass();
}

inline real_t GasSystem::velocity_y() const {
    if (n() == 0) return 0;
    else return m_state.momentum[1] / mass();
}

inline real_t GasSystem::volume() const {
    return m_state.V;
}

inline real_t GasSystem::volume(real_t n) const {
    return n * this->n() / volume();
}

inline real_t GasSystem::n_fuel() const {
    return m_state.mix.p_fuel * n();
}

inline real_t GasSystem::n_inert() const {
    return m_state.mix.p_inert * n();
}

inline real_t GasSystem::n_o2() const {
    return m_state.mix.p_o2 * n();
}

inline real_t GasSystem::heatCapacityRatio() const {
    return heatCapacityRatio(m_degreesOfFreedom);
}

#endif /* ATG_ENGINE_SIM_GAS_SYSTEM_H */
