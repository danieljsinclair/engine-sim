#include "../include/gas_system.h"

#include "../include/units.h"
#include "../include/utilities.h"

#include <cmath>
#include <cassert>

void GasSystem::setGeometry(real_t width, real_t height, real_t dx, real_t dy) {
    m_width = width;
    m_height = height;
    m_dx = dx;
    m_dy = dy;
}

void GasSystem::initialize(real_t P, real_t V, real_t T, const Mix &mix, int degreesOfFreedom) {
    m_degreesOfFreedom = degreesOfFreedom;
    m_state.n_mol = P * V / (constants::R * T);
    m_state.V = V;
    m_state.E_k = T * (0.5 * degreesOfFreedom * m_state.n_mol * constants::R);
    m_state.mix = mix;
    m_state.momentum[0] = m_state.momentum[1] = 0;

    m_chokedFlowLimit = chokedFlowLimit(degreesOfFreedom);
    m_chokedFlowFactorCached = chokedFlowRate(degreesOfFreedom);
}

void GasSystem::initialize(real_t P, real_t V, real_t T, int degreesOfFreedom) {
    initialize(P, V, T, Mix(), degreesOfFreedom);
}

void GasSystem::reset(real_t P, real_t T, const Mix &mix) {
    m_state.n_mol = P * volume() / (constants::R * T);
    m_state.E_k = T * (0.5 * m_degreesOfFreedom * m_state.n_mol * constants::R);
    m_state.mix = mix;
    m_state.momentum[0] = m_state.momentum[1] = 0;
}

void GasSystem::reset(real_t P, real_t T) {
    reset(P, T, Mix());
}

void GasSystem::setVolume(real_t V) {
    return changeVolume(V - m_state.V);
}

void GasSystem::setN(real_t n) {
    m_state.E_k = kineticEnergy(n);
    m_state.n_mol = n;
}

void GasSystem::changeVolume(real_t dV) {
    const real_t V = this->volume();
    const real_t L = std::pow(V + dV, 1 / 3.0);
    const real_t surfaceArea = (L * L);
    const real_t dL = -dV / surfaceArea;
    const real_t W = dL * pressure() * surfaceArea;

    m_state.V += dV;
    m_state.E_k += W;
}

void GasSystem::changePressure(real_t dP) {
    m_state.E_k += dP * volume() * m_degreesOfFreedom * 0.5;
}

void GasSystem::changeTemperature(real_t dT) {
    m_state.E_k += dT * 0.5 * m_degreesOfFreedom * n() * constants::R;
}

void GasSystem::changeEnergy(real_t dE) {
    m_state.E_k += dE;
}

void GasSystem::changeMix(const Mix &mix) {
    m_state.mix = mix;
}

void GasSystem::injectFuel(real_t n) {
    const real_t n_fuel = this->n_fuel() + n;
    const real_t p_fuel = n_fuel / this->n();
    m_state.mix.p_fuel = p_fuel;
}

void GasSystem::changeTemperature(real_t dT, real_t n) {
    m_state.E_k += dT * 0.5 * m_degreesOfFreedom * n * constants::R;
}

real_t GasSystem::react(real_t n, const Mix &mix) {
    const real_t l_n_fuel = mix.p_fuel * n;
    const real_t l_n_o2 = mix.p_o2 * n;

    const real_t system_n_fuel = n_fuel();
    const real_t system_n_o2 = n_o2();
    const real_t system_n_inert = n_inert();
    const real_t system_n = this->n();

    // Assuming the following reaction:
    // 25[O2] + 2[C8H16] -> 16[CO2] + 18[H2O]
    constexpr real_t ideal_o2_ratio = 25.0 / 2;
    constexpr real_t ideal_fuel_ratio = 2.0 / 25;
    constexpr real_t output_input_ratio = (16.0 + 18.0) / (25 + 2);

    const real_t ideal_fuel_n = ideal_fuel_ratio * l_n_o2;
    const real_t ideal_o2_n = ideal_o2_ratio * l_n_fuel;
    
    const real_t a_n_fuel = std::fmin(
        std::fmin(system_n_fuel, l_n_fuel),
        ideal_fuel_n);
    const real_t a_n_o2 = std::fmin(
        std::fmin(system_n_o2, l_n_o2),
        ideal_o2_n);

    const real_t reactants_n = a_n_fuel + a_n_o2;
    const real_t products_n = output_input_ratio * reactants_n;
    const real_t dn = products_n - reactants_n;

    m_state.n_mol += dn;

    // Adjust mix
    const real_t new_system_n_fuel = system_n_fuel - a_n_fuel;
    const real_t new_system_n_o2 = system_n_o2 - a_n_o2;
    const real_t new_system_n_inert = system_n_inert + products_n;
    const real_t new_system_n = system_n + dn;

    if (new_system_n != 0) {
        m_state.mix.p_fuel = new_system_n_fuel / new_system_n;
        m_state.mix.p_inert = new_system_n_inert / new_system_n;
        m_state.mix.p_o2 = new_system_n_o2 / new_system_n;
    }
    else {
        m_state.mix.p_fuel = m_state.mix.p_inert = m_state.mix.p_o2 = 0;
    }

    return a_n_fuel;
}

real_t GasSystem::flowConstant(
    real_t targetFlowRate,
    real_t P,
    real_t pressureDrop,
    real_t T,
    real_t hcr)
{
    const real_t T_0 = T;
    const real_t p_0 = P, p_T = P - pressureDrop; // p_0 = upstream pressure

    const real_t chokedFlowLimit =
        std::pow((2.0 / (hcr + 1)), hcr / (hcr - 1));
    const real_t p_ratio = p_T / p_0;

    real_t flowRate = 0;
    if (p_ratio <= chokedFlowLimit) {
        // Choked flow
        flowRate = std::sqrt(hcr);
        flowRate *= std::pow(2 / (hcr + 1), (hcr + 1) / (2 * (hcr - 1)));
    }
    else {
        flowRate = (2 * hcr) / (hcr - 1);
        flowRate *= (1 - std::pow(p_ratio, (hcr - 1) / hcr));
        flowRate = std::sqrt(flowRate);
        flowRate *= std::pow(p_ratio, 1 / hcr);
    }

    flowRate *= p_0 / std::sqrt(constants::R * T_0);

    return targetFlowRate / flowRate;
}

real_t GasSystem::k_28inH2O(real_t flowRateScfm) {
    return flowConstant(
        units::flow(flowRateScfm, units::scfm),
        units::pressure(1.0, units::atm),
        units::pressure(28.0, units::inH2O),
        units::celcius(25),
        heatCapacityRatio(5)
    );
}

real_t GasSystem::k_carb(real_t flowRateScfm) {
    return flowConstant(
        units::flow(flowRateScfm, units::scfm),
        units::pressure(1.0, units::atm),
        units::pressure(1.5, units::inHg),
        units::celcius(25),
        heatCapacityRatio(5)
    );
}

real_t GasSystem::flowRate(
    real_t k_flow,
    real_t P0,
    real_t P1,
    real_t T0,
    real_t T1,
    real_t hcr,
    real_t chokedFlowLimit,
    real_t chokedFlowRateCached)
{
    if (k_flow == 0) return 0;

    real_t direction;
    real_t T_0;
    real_t p_0, p_T; // p_0 = upstream pressure
    if (P0 > P1) {
        direction = 1.0;
        T_0 = T0;
        p_0 = P0;
        p_T = P1;
    }
    else {
        direction = -1.0;
        T_0 = T1;
        p_0 = P1;
        p_T = P0;
    }

    const real_t p_ratio = p_T / p_0;
    real_t flowRate = 0;
    if (p_ratio <= chokedFlowLimit) {
        // Choked flow
        flowRate = chokedFlowRateCached;
        flowRate /= std::sqrt(constants::R * T_0);
    }
    else {
        const real_t s = std::pow(p_ratio, 1 / hcr);

        flowRate = (2 * hcr) / (hcr - 1);
        flowRate *= s * (s - p_ratio);
        flowRate = std::sqrt(std::fmax(flowRate, 0.0) / (constants::R * T_0));
    }

    flowRate *= direction * p_0;

    return flowRate * k_flow;
}

real_t GasSystem::loseN(real_t dn, real_t E_k_per_mol) {
    m_state.E_k -= E_k_per_mol * dn;
    m_state.n_mol -= dn;

    if (m_state.n_mol < 0) {
        m_state.n_mol = 0;
    }

    return dn;
}

real_t GasSystem::gainN(real_t dn, real_t E_k_per_mol, const Mix &mix) {
    const real_t next_n = m_state.n_mol + dn;
    const real_t current_n = m_state.n_mol;

    m_state.E_k += dn * E_k_per_mol;
    m_state.n_mol = next_n;

    if (next_n != 0) {
        m_state.mix.p_fuel = (m_state.mix.p_fuel * current_n + dn * mix.p_fuel) / next_n;
        m_state.mix.p_inert = (m_state.mix.p_inert * current_n + dn * mix.p_inert) / next_n;
        m_state.mix.p_o2 = (m_state.mix.p_o2 * current_n + dn * mix.p_o2) / next_n;
    }
    else {
        m_state.mix.p_fuel = m_state.mix.p_inert = m_state.mix.p_o2 = 0;
    }

    return -dn;
}

real_t GasSystem::gainN(real_t dn, real_t E_k_per_mol) {
    return gainN(dn, E_k_per_mol, Mix());
}

void GasSystem::dissipateExcessVelocity() {
    const real_t v_x = velocity_x();
    const real_t v_y = velocity_y();
    const real_t v_squared = v_x * v_x + v_y * v_y;
    const real_t c = this->c();
    const real_t c_squared = c * c;

    if (c_squared >= v_squared || v_squared == 0) {
        return;
    }

    const real_t k_squared = c_squared / v_squared;
    const real_t k = std::sqrt(k_squared);

    m_state.momentum[0] *= k;
    m_state.momentum[1] *= k;

    m_state.E_k += 0.5 * mass() * (v_squared - c_squared);

    if (m_state.E_k < 0) m_state.E_k = 0;
}

void GasSystem::updateVelocity(real_t dt, real_t beta) {
    if (n() == 0) return;

    const real_t depth = volume() / (m_width * m_height);
    
    real_t d_momentum_x = 0;
    real_t d_momentum_y = 0;

    const real_t p0 = dynamicPressure(m_dx, m_dy);
    const real_t p1 = dynamicPressure(-m_dx, -m_dy);
    const real_t p2 = dynamicPressure(m_dy, m_dx);
    const real_t p3 = dynamicPressure(-m_dy, -m_dx);

    const real_t p_sa_0 = p0 * (m_height * depth);
    const real_t p_sa_1 = p1 * (m_height * depth);
    const real_t p_sa_2 = p2 * (m_width * depth);
    const real_t p_sa_3 = p3 * (m_width * depth);

    d_momentum_x += p_sa_0 * m_dx;
    d_momentum_y += p_sa_0 * m_dy;

    d_momentum_x -= p_sa_1 * m_dx;
    d_momentum_y -= p_sa_1 * m_dy;

    d_momentum_x += p_sa_2 * m_dy;
    d_momentum_y += p_sa_2 * m_dx;

    d_momentum_x -= p_sa_3 * m_dy;
    d_momentum_y -= p_sa_3 * m_dx;

    const real_t m = mass();
    const real_t inv_m = 1 / m;
    const real_t v0_x = m_state.momentum[0] * inv_m;
    const real_t v0_y = m_state.momentum[1] * inv_m;

    m_state.momentum[0] -= d_momentum_x * dt * beta;
    m_state.momentum[1] -= d_momentum_y * dt * beta;

    const real_t v1_x = m_state.momentum[0] * inv_m;
    const real_t v1_y = m_state.momentum[1] * inv_m;

    m_state.E_k -= 0.5 * m * (v1_x * v1_x - v0_x * v0_x);
    m_state.E_k -= 0.5 * m * (v1_y * v1_y - v0_y * v0_y);

    if (m_state.E_k < 0) m_state.E_k = 0;
}

void GasSystem::dissipateVelocity(real_t dt, real_t timeConstant) {
    if (n() == 0) return;

    const real_t invMass = 1.0 / mass();
    const real_t velocity_x = m_state.momentum[0] * invMass;
    const real_t velocity_y = m_state.momentum[1] * invMass;
    const real_t velocity_squared =
        velocity_x * velocity_x + velocity_y * velocity_y;

    const real_t s = dt / (dt + timeConstant);
    m_state.momentum[0] = m_state.momentum[0] * (1 - s);
    m_state.momentum[1] = m_state.momentum[1] * (1 - s);

    const real_t newVelocity_x = m_state.momentum[0] * invMass;
    const real_t newVelocity_y = m_state.momentum[1] * invMass;
    const real_t newVelocity_squared =
        newVelocity_x * newVelocity_x + newVelocity_y * newVelocity_y;

    const real_t dE_k = 0.5 * mass() * (velocity_squared - newVelocity_squared);
    m_state.E_k += dE_k;
}

real_t GasSystem::flow(const FlowParameters &params) {
    GasSystem *source = nullptr, *sink = nullptr;
    real_t sourcePressure = 0, sinkPressure = 0;
    real_t dx, dy;
    real_t sourceCrossSection = 0, sinkCrossSection = 0;
    real_t direction = 0;

    const real_t P_0 =
        params.system_0->pressure()
        + params.system_0->dynamicPressure(params.direction_x, params.direction_y);
    const real_t P_1 =
        params.system_1->pressure()
        + params.system_1->dynamicPressure(-params.direction_x, -params.direction_y);

    if (P_0 > P_1) {
        dx = params.direction_x;
        dy = params.direction_y;
        source = params.system_0;
        sink = params.system_1;
        sourcePressure = P_0;
        sinkPressure = P_1;
        sourceCrossSection = params.crossSectionArea_0;
        sinkCrossSection = params.crossSectionArea_1;
        direction = 1.0;
    }
    else {
        dx = -params.direction_x;
        dy = -params.direction_y;
        source = params.system_1;
        sink = params.system_0;
        sourcePressure = P_1;
        sinkPressure = P_0;
        sourceCrossSection = params.crossSectionArea_1;
        sinkCrossSection = params.crossSectionArea_0;
        direction = -1.0;
    }

    real_t flow = params.dt * flowRate(
        params.k_flow,
        sourcePressure,
        sinkPressure,
        source->temperature(),
        sink->temperature(),
        source->heatCapacityRatio(),
        source->m_chokedFlowLimit,
        source->m_chokedFlowFactorCached);

    flow = clamp(flow, real_t(0.0), real_t(0.9) * source->n());

    const real_t fraction = flow / source->n();
    const real_t fractionVolume = fraction * source->volume();
    const real_t fractionMass = fraction * source->mass();

    if (flow != 0) {
        // - Stage 1
        // Fraction flows from source to sink.

        const real_t E_k_bulk_src0 = source->bulkKineticEnergy();
        const real_t E_k_bulk_sink0 = sink->bulkKineticEnergy();

        const real_t E_k_per_mol = source->kineticEnergyPerMol();
        sink->gainN(flow, E_k_per_mol, source->mix());
        source->loseN(flow, E_k_per_mol);

        const real_t dp_x = source->m_state.momentum[0] * fraction;
        const real_t dp_y = source->m_state.momentum[1] * fraction;
        source->m_state.momentum[0] -= dp_x;
        source->m_state.momentum[1] -= dp_y;

        sink->m_state.momentum[0] += dp_x;
        sink->m_state.momentum[1] += dp_y;

        const real_t E_k_bulk_src1 = source->bulkKineticEnergy();
        const real_t E_k_bulk_sink1 = sink->bulkKineticEnergy();

        sink->m_state.E_k -= ((E_k_bulk_src1 + E_k_bulk_sink1) - (E_k_bulk_src0 + E_k_bulk_sink0));
    }
    
    const real_t sourceMass = source->mass();
    const real_t invSourceMass = 1 / sourceMass;
    const real_t sinkMass = sink->mass();
    const real_t invSinkMass = 1 / sinkMass;

    const real_t c_source = source->c();
    const real_t c_sink = sink->c();

    const real_t sourceInitialMomentum_x = source->m_state.momentum[0];
    const real_t sourceInitialMomentum_y = source->m_state.momentum[1];

    const real_t sinkInitialMomentum_x = sink->m_state.momentum[0];
    const real_t sinkInitialMomentum_y = sink->m_state.momentum[1];

    // Momentum in fraction

    if (sinkCrossSection != 0) {
        const real_t sinkFractionVelocity =
            clamp((fractionVolume / sinkCrossSection) / params.dt, real_t(0.0), c_sink);
        const real_t sinkFractionVelocity_x = sinkFractionVelocity * dx;
        const real_t sinkFractionVelocity_y = sinkFractionVelocity * dy;
        const real_t sinkFractionMomentum_x = sinkFractionVelocity_x * fractionMass;
        const real_t sinkFractionMomentum_y = sinkFractionVelocity_y * fractionMass;

        sink->m_state.momentum[0] += sinkFractionMomentum_x;
        sink->m_state.momentum[1] += sinkFractionMomentum_y;
    }

    if (sourceCrossSection != 0 && sourceMass != 0) {
        const real_t sourceFractionVelocity =
            clamp((fractionVolume / sourceCrossSection) / params.dt, real_t(0.0), c_source);
        const real_t sourceFractionVelocity_x = sourceFractionVelocity * dx;
        const real_t sourceFractionVelocity_y = sourceFractionVelocity * dy;
        const real_t sourceFractionMomentum_x = sourceFractionVelocity_x * fractionMass;
        const real_t sourceFractionMomentum_y = sourceFractionVelocity_y * fractionMass;

        source->m_state.momentum[0] += sourceFractionMomentum_x;
        source->m_state.momentum[1] += sourceFractionMomentum_y;
    }

    if (sourceMass != 0) {
        // Energy conservation
        const real_t sourceVelocity0_x = sourceInitialMomentum_x * invSourceMass;
        const real_t sourceVelocity0_y = sourceInitialMomentum_y * invSourceMass;

        const real_t sourceVelocity1_x = source->m_state.momentum[0] * invSourceMass;
        const real_t sourceVelocity1_y = source->m_state.momentum[1] * invSourceMass;

        source->m_state.E_k -=
            0.5 * sourceMass
            * (sourceVelocity1_x * sourceVelocity1_x - sourceVelocity0_x * sourceVelocity0_x);

        source->m_state.E_k -=
            0.5 * sourceMass
            * (sourceVelocity1_y * sourceVelocity1_y - sourceVelocity0_y * sourceVelocity0_y);
    }

    if (sinkMass > 0) {
        const real_t sinkVelocity0_x = sinkInitialMomentum_x * invSinkMass;
        const real_t sinkVelocity0_y = sinkInitialMomentum_y * invSinkMass;

        const real_t sinkVelocity1_x = sink->m_state.momentum[0] * invSinkMass;
        const real_t sinkVelocity1_y = sink->m_state.momentum[1] * invSinkMass;

        sink->m_state.E_k -=
            0.5 * sinkMass
            * (sinkVelocity1_x * sinkVelocity1_x - sinkVelocity0_x * sinkVelocity0_x);

        sink->m_state.E_k -=
            0.5 * sinkMass
            * (sinkVelocity1_y * sinkVelocity1_y - sinkVelocity0_y * sinkVelocity0_y);
    }

    if (sink->m_state.E_k < 0) {
        sink->m_state.E_k = 0;
    }

    if (source->m_state.E_k < 0) {
        source->m_state.E_k = 0;
    }

    return flow * direction;
}

real_t GasSystem::flow(real_t k_flow, real_t dt, real_t P_env, real_t T_env, const Mix &mix) {
    const real_t maxFlow = pressureEquilibriumMaxFlow(P_env, T_env);
    real_t flow = dt * flowRate(
        k_flow,
        pressure(),
        P_env,
        temperature(),
        T_env,
        heatCapacityRatio(),
        m_chokedFlowLimit,
        m_chokedFlowFactorCached);

    if (std::abs(flow) > std::abs(maxFlow)) {
        flow = maxFlow;
    }

    if (flow < 0) {
        const real_t bulk_E_k_0 = bulkKineticEnergy();
        gainN(-flow, kineticEnergyPerMol(T_env, m_degreesOfFreedom), mix);
        const real_t bulk_E_k_1 = bulkKineticEnergy();

        m_state.E_k += (bulk_E_k_1 - bulk_E_k_0);
    }
    else {
        const real_t starting_n = n();
        loseN(flow, kineticEnergyPerMol());

        m_state.momentum[0] -= (flow / starting_n) * m_state.momentum[0];
        m_state.momentum[1] -= (flow / starting_n) * m_state.momentum[1];
    }

    return flow;
}

real_t GasSystem::flow(real_t k_flow, real_t dt, real_t P_env, real_t T_env) {
    return flow(k_flow, dt, P_env, T_env, Mix());
}

real_t GasSystem::pressureEquilibriumMaxFlow(const GasSystem *b) const {
    // pressure_a = (kineticEnergy() + n * b->kineticEnergyPerMol()) / (0.5 * degreesOfFreedom * volume())
    // pressure_b = (b->kineticEnergy() - n *  / (0.5 * b->degreesOfFreedom * b->volume())
    // pressure_a = pressure_b

    // E_a = kineticEnergy()
    // E_b = b->kineticEnergy()
    // D_a = E_a / n()
    // D_b = E_b / b->n()
    // Q_a = 1 / (0.5 * degreesOfFreedom * volume())
    // Q_b = 1 / (0.5 * b->degreesOfFreedom * b->volume())
    // pressure_a = Q_a * (E_a + dn * D_b)
    // pressure_b = Q_b * (E_b - dn * D_b)

    if (pressure() > b->pressure()) {
        const real_t maxFlow =
                (b->volume() * kineticEnergy() - volume() * b->kineticEnergy()) /
                (b->volume() * kineticEnergyPerMol() + volume() * kineticEnergyPerMol());
        return std::fmax(0.0, std::fmin(maxFlow, n()));
    }
    else {
        const real_t maxFlow =
                (b->volume() * kineticEnergy() - volume() * b->kineticEnergy()) /
                (b->volume() * b->kineticEnergyPerMol() + volume() * b->kineticEnergyPerMol());
        return std::fmin(0.0, std::fmax(maxFlow, -b->n()));
    }
}

real_t GasSystem::pressureEquilibriumMaxFlow(real_t P_env, real_t T_env) const {
    if (pressure() > P_env) {
        return -(P_env * (0.5 * m_degreesOfFreedom * volume()) - kineticEnergy()) / kineticEnergyPerMol();
    }
    else {
        const real_t E_k_per_mol_env = 0.5 * T_env * constants::R * m_degreesOfFreedom;
        return -(P_env * (0.5 * m_degreesOfFreedom * volume()) - kineticEnergy()) / E_k_per_mol_env;
    }
}
