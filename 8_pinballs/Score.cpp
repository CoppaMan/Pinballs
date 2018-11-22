#include "Score.h"
#include <math.h>
#include <iostream>

Score::Score(Eigen::Vector3d p, int numberOfDigits) : pos(p), numDigits(numberOfDigits), score(0) {
    maxScore = pow(10,numDigits)-1;
    for(int i = 0; i < numDigits; i++) digits.emplace_back(std::make_shared<Digit>(p - i*1.5*Eigen::Vector3d::UnitX()));
}

void Score::print_score() {
    for(auto d : digits) {
        d->print_digit();
    }
}

void Score::addScore(int number) {
    score += number;
    if(score > maxScore) score = maxScore;
    number = score;

    int index = 0;
    do {
        digits[index]->update_digit(number % 10);
        number /= 10;
        index++;
    } while (number > 0);
}

void Score::emplaceInto(std::vector<std::shared_ptr<RigidObject>> *m_obj) {
    for(auto d : digits) d->emplaceInto(m_obj);
}

void Score::resetScore() {
    score = 0;
    for (auto d : digits) d->resetDigit();
}