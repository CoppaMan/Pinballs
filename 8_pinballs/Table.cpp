#include "Table.h"

Table::Table() {
    pos = Eigen::Vector3d::Zero();
    Eigen::Matrix3d tilt;
    double theta = M_PI/6.0; // tilt table 30°
    tilt << 1, 0, 0,
                0, std::cos(theta), -std::sin(theta),
                0, std::sin(theta), std::cos(theta);
    rot = tilt;

    m_table_surface =
            std::make_shared<RigidObject>("table_surface.off", ObjType::STATIC, BOUNDING_TYPE ::BOX, true);
    parts.push_back(m_table_surface);

    parts.emplace_back(std::make_shared<RigidObject>("table_wall_l.off", ObjType::STATIC));
    parts.emplace_back(std::make_shared<RigidObject>("table_wall_r.off", ObjType::STATIC));

    parts.emplace_back(std::make_shared<RigidObject>("table_bottom_l.off", ObjType::STATIC));
    parts.emplace_back(std::make_shared<RigidObject>("table_bottom_r.off", ObjType::STATIC));

    parts.emplace_back(std::make_shared<RigidObject>("table_seperator.off", ObjType::STATIC));
}

Eigen::Vector3d Table::getPosition() {
    return pos;
}

Eigen::Matrix3d Table::getRotation() {
    return rot;
}

void Table::emplaceInto(std::vector<std::shared_ptr<RigidObject>> *m_obj) {
    for(auto p : parts) m_obj->emplace_back(p);
}

void Table::resetTable() {
    for(auto p : parts) {
        p->setScale(1);
        p->setPosition(pos);
        p->setRotation(rot);
    }
}