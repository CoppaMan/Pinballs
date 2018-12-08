#include "CollisionDetection.h"

void CollisionDetection::computeBroadPhase(int broadPhaseMethod) {
    // compute possible collisions
    m_overlappingBodys.clear();
    
	switch (broadPhaseMethod) {
	case 0: { // none
		for (size_t i = 0; i < m_objects.size(); i++) {
			for (size_t j = i + 1; j < m_objects.size(); j++) {
				m_overlappingBodys.push_back(std::make_pair(i, j));
			}
		}
		break;
	}
     
	case 1: {  // AABB
        // compute bounding boxes
        std::vector<AABB> aabbs(m_objects.size());
        for (size_t i = 0; i < aabbs.size(); i++) {
            aabbs[i].computeAABB(m_objects[i]);
        }
        for (size_t i = 0; i < m_objects.size(); i++) {
            for (size_t j = i + 1; j < m_objects.size(); j++) {
                // add pair of objects to possible collision if their
                // bounding boxes overlap
                if (aabbs[i].testCollision(aabbs[j])) {
                    m_overlappingBodys.push_back(std::make_pair(i, j));
                }
            }
        }
        break;
    }
	
	case 2: {
		// TODO
		break;
	}
	}
}

void CollisionDetection::computeNarrowPhase(int narrowPhaseMethod) {
    switch (narrowPhaseMethod) {
    case 0: {
   
        break;
    }

	case 1: {
		// TODO
		break;
	}
    }
}

void CollisionDetection::applyImpulse(double eps) {
    // compute impulse for all contacts
    for (auto contact : m_contacts) {
        Eigen::Vector3d vrel_vec = contact.a->getVelocity(contact.p) -
                                    contact.b->getVelocity(contact.p);
        double vrel = contact.n.dot(vrel_vec);
        if (vrel > 0) {
            // bodies are moving apart
            continue;
        }

		// compute impulse and update the following momentums

		//contact.a->setLinearMomentum();
		//contact.b->setLinearMomentum();
		//contact.a->setAngularMomentum();
		//contact.b->setAngularMomentum();
    }
}