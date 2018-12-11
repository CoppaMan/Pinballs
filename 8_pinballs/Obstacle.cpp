#include "Obstacle.h"
#include <string>

Obstacle::Obstacle(std::shared_ptr<Table> table, std::string name, int count, Eigen::Vector3d pos, Eigen::Vector3d color, double angle, bool mirror) : table(table), pos_rel(pos), color(color), rot_rel(angle), mirrored(mirror) {
    pos_abs = table->getPosition() + (table->getRotation()*pos_rel);

    Eigen::Matrix3d spin;
    spin << cos(angle), 0, sin(angle),
                0, 1, 0,
                -sin(angle), 0, cos(angle);

    Eigen::Matrix3d flip;
    double phi = M_PI; 
    flip << cos(phi), sin(phi), 0,
            -sin(phi), cos(phi), 0,
            0, 0, 1;

    
    rot_abs = mirror ? table->getRotation()*spin*flip : table->getRotation()*spin;

    for(int i = 0; i < count; i++) {
        std::stringstream path;
        path << name << "_" << i << ".off";
        parts.emplace_back(std::make_shared<RigidObject>(path.str(), ObjType::STATIC));
    }
}

void Obstacle::emplaceInto(std::vector<std::shared_ptr<RigidObject>> *m_obj) {
    for(auto p : parts) m_obj->emplace_back(p);
}

void Obstacle::resetObstacle() {
    for(auto p : parts) {
        p->setPosition(pos_abs);
        Eigen::MatrixXd col(1,3);
        col << color(0), color(1), color(2);
        p->setColors(col);
        p->setRotation(rot_abs);
    }
}

void Obstacle::addEffect(std::shared_ptr<Effect> e) {
    std::cout << "Adding effect " << std::endl;
    for(auto p : parts) {
        p->addEffect(e);
        e->addObject(p);
    }
}

void Obstacle::printDebug() {
    for(int i = 0; i < parts.size(); i++) {
        std::cout << "Part " << i << ":" << std::endl;
        parts.at(i)->printDebug();
    } 
}