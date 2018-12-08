#pragma once

#include "RigidObject.h"
#include <vector>

class Table {
    public:
        Table();
        void emplaceInto(std::vector<std::shared_ptr<RigidObject>> *m_obj);
        void resetTable();
        Eigen::Vector3d getPosition();
        Eigen::Matrix3d getRotation();
        std::shared_ptr<RigidObject> m_table_surface;

    private:
        std::vector<std::shared_ptr<RigidObject>> parts;

        Eigen::Vector3d pos;
        Eigen::Matrix3d rot;
};