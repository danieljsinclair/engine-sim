#ifndef ATG_ENGINE_FUEL_H
#define ATG_ENGINE_FUEL_H

#include "units.h"

#include "function.h"

#include <string>

class Fuel {
    public:
        struct Parameters {
            std::string name = "Gasoline";
            double molecularMass =
                units::mass(100.0, units::g);
            double energyDensity =
                units::energy(48.1, units::kJ) / units::mass(1.0, units::g);
            double density =
                units::mass(0.755, units::kg) / units::volume(1.0, units::L);
            double molecularAfr = 25 / 2.0;
            double burningEfficiencyRandomness = 0.5;
            double lowEfficiencyAttenuation = 0.6;
            double maxBurningEfficiency = 0.8;
            double maxTurbulenceEffect = 2.0;
            double maxDilutionEffect = 50.0;
            Function *turbulenceToFlameSpeedRatio = nullptr;
        };

        Fuel();
        ~Fuel();

        void initialize(const Parameters &params);

        inline real_t getMolecularMass() const { return m_molecularMass; }
        inline real_t getEnergyDensity() const { return m_energyDensity; }
        inline real_t getDensity() const { return m_density; }
        inline real_t getBurningEfficiencyRandomness() const { return m_burningEfficiencyRandomness; }
        inline real_t getLowEfficiencyAttenuation() const { return m_lowEfficiencyAttenuation;  }
        inline real_t getMaxBurningEfficiency() const { return m_maxBurningEfficiency; }
        inline real_t getMaxTurbulenceEffect() const { return m_maxTurbulenceEffect; }
        inline real_t getMaxDilutionEffect() const { return m_maxDilutionEffect; }

        real_t flameSpeed(
            real_t turbulence,
            real_t molecularAfr,
            real_t T,
            real_t P,
            real_t firingPressure,
            real_t motoringPressure) const;
        virtual real_t laminarBurningVelocity(real_t molecularAfr, real_t T, real_t P) const;

        real_t getMolecularAfr() const { return m_molecularAfr; }

    protected:
        std::string m_name;
        real_t m_molecularMass;
        real_t m_energyDensity;
        real_t m_density;
        real_t m_molecularAfr;
        real_t m_maxBurningEfficiency;
        real_t m_burningEfficiencyRandomness;
        real_t m_lowEfficiencyAttenuation;
        real_t m_maxTurbulenceEffect;
        real_t m_maxDilutionEffect;

        Function *m_turbulenceToFlameSpeedRatio;
};

#endif /* ATG_ENGINE_FUEL_H */
