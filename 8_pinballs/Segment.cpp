#include "Segment.h"

Segment::Segment(Eigen::Vector3d p, bool is_horizontal) : RigidObject("segment.off", ObjType::INTANGIBLE), pos(p), horizontal(is_horizontal) {}

void Segment::resetSegment(bool is_on) {
    toggleSegment(is_on);
    setPosition(pos);
    resetMembers();
    if(horizontal) {
        Eigen::Matrix3d rotation_z;
        double theta = M_PI/2.0; //rotate around FRONT (z-axis) counter-clock wise
        rotation_z << std::cos(theta), -std::sin(theta), 0,
                std::sin(theta), std::cos(theta), 0,
                0, 0, 1;
        setRotation(rotation_z);
    }
    setScale(0.5);
}

void Segment::toggleSegment(bool state) {
    is_active = state;
    Eigen::MatrixXd color(1, 3);
    color << (is_active ? 1.0 : 0.0), 0.0, 0.0;
    setColors(color);
}