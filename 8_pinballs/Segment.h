#pragma once

#include "RigidObject.h"

class Segment : public RigidObject {
    public:
        Segment(Eigen::Vector3d p, bool is_horizontal);
        void resetSegment(bool is_on);
        void toggleSegment(bool state);
    private:
        bool is_active;
        bool horizontal;
        RigidObject* seg;
        Eigen::Vector3d pos;
};