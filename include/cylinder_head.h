#ifndef ATG_ENGINE_SIM_CYLINDER_HEAD_H
#define ATG_ENGINE_SIM_CYLINDER_HEAD_H

#include "part.h"

#include "function.h"
#include "camshaft.h"
#include "exhaust_system.h"
#include "intake.h"

class Valvetrain;
class CylinderBank;
class CylinderHead : public Part {
    public:
        struct Parameters {
            CylinderBank *Bank;

            Function *ExhaustPortFlow;
            Function *IntakePortFlow;

            Valvetrain *Valvetrain;

            double CombustionChamberVolume;

            double IntakeRunnerVolume;
            double IntakeRunnerCrossSectionArea;
            double ExhaustRunnerVolume;
            double ExhaustRunnerCrossSectionArea;

            bool FlipDisplay = false;
        };

        struct Cylinder {
            ExhaustSystem *exhaustSystem = nullptr;
            Intake *intake = nullptr;

            real_t soundAttenuation = 1.0;
            real_t headerPrimaryLength = 0.0;
        };

    public:
        CylinderHead();
        virtual ~CylinderHead();

        void initialize(const Parameters &params);
        virtual void destroy();

        real_t intakeFlowRate(int cylinder) const;
        real_t exhaustFlowRate(int cylinder) const;
        real_t intakeValveLift(int cylinder) const;
        real_t exhaustValveLift(int cylinder) const;

        inline ExhaustSystem *getExhaustSystem(int cylinderIndex) const { return m_cylinders[cylinderIndex].exhaustSystem; }
        void setAllExhaustSystems(ExhaustSystem *system);
        void setExhaustSystem(int i, ExhaustSystem *system);

        inline real_t getSoundAttenuation(int cylinderIndex) const { return m_cylinders[cylinderIndex].soundAttenuation; }
        void setSoundAttenuation(int i, real_t soundAttenuation);

        inline Intake *getIntake(int cylinderIndex) const { return m_cylinders[cylinderIndex].intake; }
        void setAllIntakes(Intake *intake);
        void setIntake(int i, Intake *intake);

        inline real_t getHeaderPrimaryLength(int cylinderIndex) const { return m_cylinders[cylinderIndex].headerPrimaryLength; }
        void setAllHeaderPrimaryLengths(real_t length);
        void setHeaderPrimaryLength(int i, real_t length);

        inline bool getFlipDisplay() const { return m_flipDisplay; }
        inline real_t getCombustionChamberVolume() const { return m_combustionChamberVolume; }
        inline CylinderBank *getCylinderBank() const { return m_bank; }

        real_t getIntakeRunnerVolume() const { return m_intakeRunnerVolume; }
        real_t getIntakeRunnerCrossSectionArea() const { return m_intakeRunnerCrossSectionArea; }
        real_t getExhaustRunnerVolume() const { return m_exhaustRunnerVolume; }
        real_t getExhaustRunnerCrossSectionArea() const { return m_exhaustRunnerCrossSectionArea; }

        Camshaft *getExhaustCamshaft();
        Camshaft *getIntakeCamshaft();

    protected:
        Cylinder *m_cylinders;

        CylinderBank *m_bank;
        Valvetrain *m_valvetrain;

        Function *m_exhaustPortFlow;
        Function *m_intakePortFlow;

        real_t m_intakeRunnerVolume;
        real_t m_intakeRunnerCrossSectionArea;
        real_t m_exhaustRunnerVolume;
        real_t m_exhaustRunnerCrossSectionArea;

        real_t m_combustionChamberVolume;
        bool m_flipDisplay;
};

#endif /* ATG_ENGINE_SIM_CYLINDER_HEAD_H */
