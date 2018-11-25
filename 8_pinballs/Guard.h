#pragma once

#include <memory>
#include "RigidObject.h"
#include "Table.h"

class Guard {
    public:
        Guard(std::shared_ptr<Table> table, Eigen::Vector3d pos, double angle);
        void emplaceInto(std::vector<std::shared_ptr<RigidObject>> *m_obj);
        void resetGuard();
    private:
        std::shared_ptr<Table> parent;
        std::vector<std::shared_ptr<RigidObject>> parts;
        Eigen::Vector3d pos_rel;
        Eigen::Vector3d pos_abs;
        double rot_rel;
        Eigen::Matrix3d rot_abs;
};