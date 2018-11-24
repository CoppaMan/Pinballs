#include "ScoreEffect.h"
#include "PinballSim.h"

ScoreEffect::ScoreEffect(PinballSim *sim, long new_v) : value(new_v), Effect(sim) {}

void ScoreEffect::apply() {
    ps->score->addScore(value);
}