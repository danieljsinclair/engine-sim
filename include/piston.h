#ifndef ATG_ENGINE_SIM_PISTON_H
#define ATG_ENGINE_SIM_PISTON_H

#include "part.h"

class ConnectingRod;
class CylinderBank;
class Piston : public Part {
    public:
        struct Parameters {
            ConnectingRod *Rod;
            CylinderBank *Bank;
            int CylinderIndex;

            double BlowbyFlowCoefficient;
            double CompressionHeight;
            double WristPinPosition;
            double Displacement;
            double mass;
        };

    public:
        Piston();
        virtual ~Piston();

        void initialize(const Parameters &params);
        inline void setCylinderConstraint(atg_scs::LineConstraint *constraint);
        virtual void destroy();

        real_t relativeX() const;
        real_t relativeY() const;

        real_t calculateCylinderWallForce() const;
        inline ConnectingRod *getRod() const { return m_rod; }
        inline CylinderBank *getCylinderBank() const { return m_bank; }
        inline int getCylinderIndex() const { return m_cylinderIndex; }
        inline real_t getCompressionHeight() const { return m_compressionHeight; }
        inline real_t getDisplacement() const { return m_displacement; }
        inline real_t getWristPinLocation() const { return m_wristPinLocation; }
        inline real_t getMass() const { return m_mass; }
        inline real_t getBlowbyK() const { return m_blowby_k; }

    protected:
        ConnectingRod *m_rod;
        CylinderBank *m_bank;
        atg_scs::LineConstraint *m_cylinderConstraint;
        int m_cylinderIndex;
        real_t m_compressionHeight;
        real_t m_displacement;
        real_t m_wristPinLocation;
        real_t m_mass;
        real_t m_blowby_k;
};

void Piston::setCylinderConstraint(atg_scs::LineConstraint *constraint) {
    m_cylinderConstraint = constraint;
}

#endif /* ATG_ENGINE_SIM_PISTON_H */
