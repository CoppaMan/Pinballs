#include <Effect.h>

class ScoreEffect : public Effect {
    public:
        ScoreEffect(PinballSim sim, long new_v);
        void apply() override;
    private:
        long value;
};