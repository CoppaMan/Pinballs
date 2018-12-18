#include "ScoreEffect.h"
#include "PinballSim.h"

ScoreEffect::ScoreEffect(PinballSim *sim, long new_v) : Effect(sim, 0.1), value(new_v) {}

void ScoreEffect::run(std::shared_ptr<RigidObject> other) {
    std::cout << "Increased score by " << value << std::endl;
    ps->score->addScore(value);
}