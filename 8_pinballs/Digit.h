#pragma once

#include "Segment.h"
#include <memory>
#include <vector>

class Digit {
    public:
        Digit(Eigen::Vector3d p);
        void update_digit(int digit);
        void print_digit();
        void emplaceInto(std::vector<std::shared_ptr<RigidObject>> *m_obj);
        void resetDigit();
    private:
        int digit = 0;
        std::vector<std::shared_ptr<Segment>> segments;
        bool bitmap[7];
        Eigen::Vector3d pos;
        void allTrue(bool bm[]);
        void allFalse(bool bm[]);
};