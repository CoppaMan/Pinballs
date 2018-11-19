#ifndef BOUNDINGBOX
#define BOUNDINGBOX

#include "BoundingObject.h"
#include <Eigen/Core>

class BoundingBox : public BoundingObject {

public:
    BoundingBox(const Eigen::Vector3d& minCoord, const Eigen::Vector3d& maxCoord)
            : BoundingObject(BOUNDING_TYPE::BOX), m_minCoord(minCoord), m_maxCoord(maxCoord) {}


    BoundingBox(Eigen::MatrixXd &V) : BoundingObject(BOUNDING_TYPE::BOX) {
        Eigen::Vector3d minCoord = V.colwise().minCoeff();
        Eigen::Vector3d maxCoord = V.colwise().maxCoeff();

        m_minCoord = minCoord;
        m_maxCoord = maxCoord;
    }

    Eigen::Vector3d m_minCoord;
    Eigen::Vector3d m_maxCoord;
};

#endif // BOUNDINGBOX