#include "Guard.h"
#include <string>

Guard::Guard(std::shared_ptr<Table> table, Eigen::Vector3d pos, double angle) : parent(table), pos_rel(pos), rot_rel(angle) {
    pos_abs = pos;

    Eigen::Matrix3d rotation_y;
    rotation_y << cos(angle), 0, sin(angle),
                0, 1, 0,
                -sin(angle), 0, cos(angle);
    rot_abs = rotation_y;

    for(int i = 0; i < 6; i++) {
        std::stringstream name;
        name << "guard_" << i << ".off";
        parts.emplace_back(std::make_shared<RigidObject>(name.str(), ObjType::DYNAMIC));
    }
}

void Guard::emplaceInto(std::vector<std::shared_ptr<RigidObject>> *m_obj) {
    for(auto p : parts) m_obj->emplace_back(p);
}

void Guard::resetGuard() {
    for(auto p : parts) {
        p->setPosition(pos_abs);
        p->setRotation(rot_abs);
    }
}