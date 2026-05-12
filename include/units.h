#ifndef ATG_ENGINE_SIM_UNITS_H
#define ATG_ENGINE_SIM_UNITS_H

#include "constants.h"

namespace units {
    // Force
    inline constexpr real_t N = 1.0;

    inline constexpr real_t lbf = N * 4.44822;

    // Mass
    inline constexpr real_t kg = 1.0;
    inline constexpr real_t g = kg / 1000.0;

    inline constexpr real_t lb = 0.45359237 * kg;

    // Distance
    inline constexpr real_t m = 1.0;
    inline constexpr real_t cm = m / 100.0;
    inline constexpr real_t mm = m / 1000.0;
    inline constexpr real_t km = m * 1000.0;

    inline constexpr real_t inch = cm * 2.54;
    inline constexpr real_t foot = inch * 12.0;
    inline constexpr real_t thou = inch / 1000.0;
    inline constexpr real_t mile = m * 1609.344;

    // Time
    inline constexpr real_t sec = 1.0;
    inline constexpr real_t minute = 60 * sec;
    inline constexpr real_t hour = 60 * minute;

    // Torque
    inline constexpr real_t Nm = N * m;
    inline constexpr real_t ft_lb = foot * lbf;

    // Power
    inline constexpr real_t W = Nm / sec;
    inline constexpr real_t kW = W * 1000.0;
    inline constexpr real_t hp = 745.699872 * W;

    // Volume
    inline constexpr real_t m3 = 1.0;
    inline constexpr real_t cc = cm * cm * cm;
    inline constexpr real_t mL = cc;
    inline constexpr real_t L = mL * 1000.0;
    inline constexpr real_t cubic_feet = foot * foot * foot;
    inline constexpr real_t cubic_inches = inch * inch * inch;
    inline constexpr real_t gal = 3.785411784 * L;

    // Molecular
    inline constexpr real_t mol = 1.0;
    inline constexpr real_t kmol = mol / 1000.0;
    inline constexpr real_t mmol = mol / 1000000.0;
    inline constexpr real_t lbmol = mol * 453.59237;

    // Flow-rate (moles)
    inline constexpr real_t mol_per_sec = mol / sec;
    inline constexpr real_t scfm = 0.002641 * lbmol / minute;

    // Area
    inline constexpr real_t m2 = 1.0;
    inline constexpr real_t cm2 = cm * cm;

    // Pressure
    inline constexpr real_t Pa = 1.0;
    inline constexpr real_t kPa = Pa * 1000.0;
    inline constexpr real_t MPa = Pa * 1000000.0;
    inline constexpr real_t atm = 101.325 * kPa;

    inline constexpr real_t mbar = Pa * 100.0;
    inline constexpr real_t bar = mbar * 1000.0;

    inline constexpr real_t psi = lbf / (inch * inch);
    inline constexpr real_t psig = psi;
    inline constexpr real_t inHg = Pa * 3386.3886666666713;
    inline constexpr real_t inH2O = inHg * 0.0734824;

    // Temperature
    inline constexpr real_t K = 1.0;
    inline constexpr real_t K0 = 273.15;
    inline constexpr real_t C = K;
    inline constexpr real_t F = (5.0 / 9.0) * K;
    inline constexpr real_t F0 = -459.67;

    // Energy
    inline constexpr real_t J = 1.0;
    inline constexpr real_t kJ = J * 1000;
    inline constexpr real_t MJ = J * 1000000;

    // Angles
    inline constexpr real_t rad = 1.0;
    inline constexpr real_t deg = rad * (constants::pi / 180);

    // Conversions
    inline constexpr real_t distance(real_t v, real_t unit) {
        return v * unit;
    }

    inline constexpr real_t area(real_t v, real_t unit) {
        return v * unit;
    }

    inline constexpr real_t torque(real_t v, real_t unit) {
        return v * unit;
    }

    inline constexpr real_t rpm(real_t rpm) {
        return rpm * 0.104719755;
    }

    inline constexpr real_t toRpm(real_t rad_s) {
        return rad_s / 0.104719755;
    }

    inline constexpr real_t pressure(real_t v, real_t unit) {
        return v * unit;
    }

    inline constexpr real_t psia(real_t p) {
        return units::pressure(p, units::psig) - units::pressure(1.0, units::atm);
    }

    inline constexpr real_t toPsia(real_t p) {
        return (p + units::pressure(1.0, units::atm)) / units::psig;
    }

    inline constexpr real_t mass(real_t v, real_t unit) {
        return v * unit;
    }

    inline constexpr real_t force(real_t v, real_t unit) {
        return v * unit;
    }

    inline constexpr real_t volume(real_t v, real_t unit) {
        return v * unit;
    }

    inline constexpr real_t flow(real_t v, real_t unit) {
        return v * unit;
    }

    inline constexpr real_t convert(real_t v, real_t unit0, real_t unit1) {
        return v * (unit0 / unit1);
    }

    inline constexpr real_t convert(real_t v, real_t unit) {
        return v / unit;
    }

    inline constexpr real_t celcius(real_t T_C) {
        return T_C * C + K0;
    }

    inline constexpr real_t kelvin(real_t T) {
        return T * K;
    }

    inline constexpr real_t fahrenheit(real_t T_F) {
        return F * (T_F - F0);
    }

    inline constexpr real_t toAbsoluteFahrenheit(real_t T) {
        return T / F;
    }

    inline constexpr real_t angle(real_t v, real_t unit) {
        return v * unit;
    }

    inline constexpr real_t energy(real_t v, real_t unit) {
        return v * unit;
    }

    // Physical Constants
    constexpr real_t AirMolecularMass = units::mass(28.97, units::g) / units::mol;
};

#endif /* ATG_ENGINE_SIM_UNITS_H */
