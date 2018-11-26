#pragma once

#include <RigidObject.h>

class Table : public RigidObject {
public:
    Table() : RigidObject("table_surface.off") {
        resetTable();
    }

    void resetTable() {
        setScale(1);
        setType(ObjType::STATIC);
        setPosition(Eigen::Vector3d(0, 0, 0));

        Eigen::Matrix3d tilt;
        double theta = M_PI/6.0; // tilt table 30°
        tilt << 1, 0, 0,
                    0, std::cos(theta), -std::sin(theta),
                    0, std::sin(theta), std::cos(theta);
                
        setRotation(tilt);
    }
};