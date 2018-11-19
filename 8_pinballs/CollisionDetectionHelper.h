#ifndef COLDETHELPER
#define COLDETHELPER


#include "BoundingSphere.h"
#include <memory>
#include "AABB.h"

class CollisionDetectionHelper {
public:
    static inline bool isCollision(const std::shared_ptr<BoundingSphere> s1, const std::shared_ptr<BoundingSphere> s2) {
        Eigen::Vector3d offset = s1->m_center - s2->m_center;
        return offset.squaredNorm() <= (s1->m_radius + s2->m_radius)*(s1->m_radius + s2->m_radius);
    }

    static inline bool isCollision(const std::shared_ptr<BoundingSphere> s1, const std::shared_ptr<BoundingBox> b1) {
        Eigen::Vector3d nearest_point = b1->m_minCoord.cwiseMax(s1->m_center.cwiseMin(b1->m_maxCoord));
        return nearest_point.norm() < s1->m_radius;
    }

    static inline bool isCollision(const std::shared_ptr<BoundingBox> b1, const std::shared_ptr<BoundingBox> b2) {
        if (b1->m_maxCoord.x() < b2->m_minCoord.x() ||
            b2->m_maxCoord.x() < b1->m_minCoord.x())
            return false;
        if (b1->m_maxCoord.y() < b2->m_minCoord.y() ||
            b2->m_maxCoord.y() < b1->m_minCoord.y())
            return false;
        if (b1->m_maxCoord.z() < b2->m_minCoord.z() ||
            b2->m_maxCoord.z() < b1->m_minCoord.z())
            return false;
        return true;
    }

};

#endif // COLDETHELPER