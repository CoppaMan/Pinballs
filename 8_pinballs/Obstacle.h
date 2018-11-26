#pragma once

#include <memory>
#include <string>
#include "RigidObject.h"
#include "Table.h"

class Obstacle {
    public:
        Obstacle(std::shared_ptr<Table> table, std::string name, int count, Eigen::Vector3d pos, double angle, bool mirror);
        void emplaceInto(std::vector<std::shared_ptr<RigidObject>> *m_obj);
        void resetObstacle();
        void addEffect(std::shared_ptr<Effect> effect);
    private:
        int count;
        std::shared_ptr<Table> table;
        std::vector<std::shared_ptr<RigidObject>> parts;
        Eigen::Vector3d pos_rel;
        Eigen::Vector3d pos_abs;
        double rot_rel;
        Eigen::Matrix3d rot_abs;
        bool mirrored;
};