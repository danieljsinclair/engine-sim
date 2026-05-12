#ifndef ATG_ENGINE_CONNECTING_ROD_H
#define ATG_ENGINE_CONNECTING_ROD_H

#include "part.h"

#include "crankshaft.h"

class Piston;
class ConnectingRod : public Part {
    public:
        struct Parameters {
            double mass = 0.0;
            double momentOfInertia = 0.0;
            double centerOfMass = 0.0;
            double length = 0.0;

            int rodJournals = 0;
            double slaveThrow = 0;

            Piston *piston = nullptr;

            Crankshaft *crankshaft = nullptr;
            ConnectingRod *master = nullptr;
            int journal = 0;
        };

    public:
        ConnectingRod();
        virtual ~ConnectingRod();

        void initialize(const Parameters &params);

        real_t getBigEndLocal() const;
        real_t getLittleEndLocal() const;

        void setMaster(ConnectingRod *rod) { m_master = rod; }
        void setCrankshaft(Crankshaft *crank) { m_crankshaft = crank; }

        inline int getRodJournalCount() const { return m_rodJournalCount; }
        void setRodJournalAngle(int i, real_t angle);
        void getRodJournalPositionLocal(int i, real_t *x, real_t *y);
        void getRodJournalPositionGlobal(int i, real_t *x, real_t *y);
        real_t getRodJournalAngle(int i) { return m_rodJournalAngles[i]; }

        inline real_t getSlaveThrow() const { return m_slaveThrow; }
        inline real_t getCenterOfMass() const { return m_centerOfMass; }
        inline real_t getLength() const { return m_length; }
        inline real_t getMass() const { return m_m; }
        inline real_t getMomentOfInertia() const { return m_I; }
        inline int getJournal() const { return m_journal; }
        int getLayer() const;
        inline ConnectingRod *getMasterRod() const { return m_master; }
        inline Crankshaft *getCrankshaft() const { return m_crankshaft; }
        inline Piston *getPiston() const { return m_piston; }

    protected:
        real_t m_centerOfMass;
        real_t m_length;
        real_t m_m;
        real_t m_I;
        int m_journal;
        ConnectingRod *m_master;
        Crankshaft *m_crankshaft;
        Piston *m_piston;

        real_t m_slaveThrow;
        real_t *m_rodJournalAngles;
        int m_rodJournalCount;
};

#endif /* ATG_ENGINE_SIM_CONNECTING_ROD_H */
