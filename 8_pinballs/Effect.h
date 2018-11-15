#include <PinballSim.h>

class Effect {
    public:
        Effect(PinballSim sim);
        virtual void apply();

    protected:
        PinballSim *ps;
        RigidObject obj;
};