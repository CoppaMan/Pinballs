#include "ScoreEffect.h"
#include "PinballSim.h"

ScoreEffect::ScoreEffect(PinballSim *sim, long new_v) : Effect(sim, 0.5), value(new_v) {}

void ScoreEffect::run() {
    std::cout << "Increased score by " << value << std::endl;
    ps->score->addScore(value);
}