#ifndef BLABLA
#define BLABLA

#include <Eigen/Core>
#include "BoundingObject.h"


class BoundingSphere : public BoundingObject {
public:
    BoundingSphere(const Eigen::Vector3d &center, double radius) :
            BoundingObject(BOUNDING_TYPE::SPHERE), m_center(center), m_radius(radius) {}

    BoundingSphere(Eigen::MatrixXd &V): BoundingObject(BOUNDING_TYPE::SPHERE) {
        // compute bounding box first
        Eigen::Vector3d minCoord = V.colwise().minCoeff();
        Eigen::Vector3d maxCoord = V.colwise().maxCoeff();

        m_center = (minCoord + maxCoord) /2;

        // currently we just take a random point to calculate the radius from the center
        // since we know we are a pinball (if you need to use this for other rigid objects consider
        // updating this
        m_radius = (V.row(0).transpose() - m_center).norm();
    }


    Eigen::Vector3d m_center;
    double m_radius;

};

#endif