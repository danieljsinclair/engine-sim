#ifndef ATG_ENGINE_SIM_EXHAUST_SYSTEM_H
#define ATG_ENGINE_SIM_EXHAUST_SYSTEM_H

#include "part.h"

#include "gas_system.h"
#include "impulse_response.h"

class ExhaustSystem : public Part {
    friend class Engine;

    public:
        struct Parameters {
            double length;
            double collectorCrossSectionArea;
            double outletFlowRate;
            double primaryTubeLength;
            double primaryFlowRate;
            double velocityDecay;
            double audioVolume;
            ImpulseResponse *impulseResponse;
        };

    public:
        ExhaustSystem();
        virtual ~ExhaustSystem();

        void initialize(const Parameters &params);
        virtual void destroy();

        void process(real_t dt);

        inline int getIndex() const { return m_index; }
        inline real_t getLength() const { return m_length; }
        inline real_t getFlow() const { return m_flow; }
        inline real_t getAudioVolume() const { return m_audioVolume; }
        inline real_t getPrimaryFlowRate() const { return m_primaryFlowRate; }
        inline real_t getCollectorCrossSectionArea() const { return m_collectorCrossSectionArea; }
        inline real_t getPrimaryTubeLength() const { return m_primaryTubeLength; }
        inline real_t getVelocityDecay() const { return m_velocityDecay; }
        inline ImpulseResponse *getImpulseResponse() const { return m_impulseResponse; }

        inline GasSystem *getSystem() { return &m_system; }

    protected:
        GasSystem m_atmosphere;
        GasSystem m_system;

        ImpulseResponse *m_impulseResponse;

        real_t m_length;
        real_t m_primaryTubeLength;
        real_t m_collectorCrossSectionArea;
        real_t m_primaryFlowRate;
        real_t m_outletFlowRate;
        real_t m_audioVolume;
        real_t m_velocityDecay;
        int m_index;

        real_t m_flow;
};

#endif /* ATG_ENGINE_SIM_EXHAUST_SYSTEM_H */
