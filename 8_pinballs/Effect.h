#include <igl/edges.h>

#ifndef Included_Effect
#define Included_Effect

// forward declr
class PinballSim;

class Effect {
    public:
        Effect(PinballSim *sim);
        virtual void apply();

    protected:
        PinballSim *ps;
};

# endif // Included_Effect