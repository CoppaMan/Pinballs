#include <vector>
#include <memory>
#include "Digit.h"
#include "RigidObject.h"

class Score {
    public:
        Score(Eigen::Vector3d p, int numberOfDigits);
        void addScore(int number);
        void print_score();
        void emplaceInto(std::vector<std::shared_ptr<RigidObject>> *m_obj);
        void resetScore();
    private:
        Eigen::Vector3d pos;
        int numDigits;
        std::vector<std::shared_ptr<Digit>> digits; //lowest digit first
        long score;
        long maxScore;
};