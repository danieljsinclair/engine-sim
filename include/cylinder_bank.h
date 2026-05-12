#ifndef ATG_ENGINE_SIM_CYLINDER_BANK_H
#define ATG_ENGINE_SIM_CYLINDER_BANK_H

#include "part.h"
#include "types.h"

#include "crankshaft.h"

class CylinderBank {
    public:
        struct Parameters {
            Crankshaft *crankshaft;
            double positionX;
            double positionY;
            double angle;
            double bore;
            double deckHeight;
            double displayDepth;
            int cylinderCount;
            int index;
        };

    public:
        CylinderBank();
        ~CylinderBank();

        void initialize(const Parameters &params);
        void destroy();

        void getPositionAboveDeck(real_t h, real_t *x, real_t *y) const;
        real_t boreSurfaceArea() const;

        inline real_t getAngle() const { return m_angle; }
        inline real_t getBore() const { return m_bore; }
        inline real_t getDeckHeight() const { return m_deckHeight; }
        inline int getCylinderCount() const { return m_cylinderCount; }
        inline int getIndex() const { return m_index; }
        inline real_t getDx() const { return m_dx; }
        inline real_t getDy() const { return m_dy; }
        inline real_t getX() const { return m_x; }
        inline real_t getY() const { return m_y; }
        inline real_t getDisplayDepth() const { return m_displayDepth; }

    protected:
        real_t m_angle;
        real_t m_bore;
        real_t m_deckHeight;
        real_t m_displayDepth;
        int m_cylinderCount;
        int m_index;

        real_t m_dx;
        real_t m_dy;
        real_t m_x;
        real_t m_y;
};

#endif /* ATG_ENGINE_SIM_CYLINDER_BANK_H */
