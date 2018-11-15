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
        // exhaustive
        // iterate through all pairs of possible collisions
        for (auto overlap : m_overlappingBodys) {
            std::vector<Contact> temp_contacts[2];
            // compute intersection of a with b first and intersectino
            // of b with a and store results in temp_contacts
            for (int switcher = 0; switcher < 2; switcher++) {
                RigidObject* a =
                    &m_objects[(!switcher) ? overlap.first
                                            : overlap.second];
                RigidObject* b =
                    &m_objects[(!switcher) ? overlap.second
                                            : overlap.first];

                Eigen::MatrixXd Va, Vb;
                Eigen::MatrixXi Fa, Fb;
                a->getMesh(Va, Fa);
                b->getMesh(Vb, Fb);

                // iterate through all faces of first object
                for (int face = 0; face < Fa.rows(); face++) {
                    // iterate through all edges of given face
                    for (size_t j = 0; j < 3; j++) {
                        int start = Fa(face, j);
                        int end = Fa(face, (j + 1) % 3);

                        // check if there is a collision
                        ContactType ct = isColliding(
                            Va.row(start), Va.row(end), Vb, Fb);

                        // find collision and check for duplicates
                        switch (ct) {
                            case ContactType::VERTEXFACE: {
                                auto ret = m_penetratingVertices.insert(
                                    Fa(face, j));
                                // if not already in set
                                if (ret.second) {
                                    Contact temp_col =
                                        findVertexFaceCollision(
                                            Va.row(Fa(face, j)), Vb,
                                            Fb);
                                    temp_col.a = a;
                                    temp_col.b = b;
                                    temp_col.type =
                                        ContactType::VERTEXFACE;
                                    temp_contacts[switcher].push_back(
                                        temp_col);
                                }
                                break;
                            }
                            case ContactType::EDGEEDGE: {
                                int orderedStart = std::min(start, end);
                                int orderedEnd = std::max(start, end);
                                auto ret = m_penetratingEdges.insert(
                                    std::make_pair(orderedStart,
                                                    orderedEnd));
                                // if not already in set
                                if (ret.second) {
                                    Contact temp_col =
                                        findEdgeEdgeCollision(
                                            Va.row(orderedStart),
                                            Va.row(orderedEnd), Vb, Fb);
                                    temp_col.a = a;
                                    temp_col.b = b;
                                    temp_col.type =
                                        ContactType::EDGEEDGE;
                                    temp_contacts[switcher].push_back(
                                        temp_col);
                                }
                                break;
                            }
                            case ContactType::NONE: {
                                break;
                            }
                        }
                    }
                }
            }

            // look for vertexFace
            bool found = false;
            for (int i = 0; i < 2; i++) {
                for (auto cont : temp_contacts[i]) {
                    if (cont.type == ContactType::VERTEXFACE) {
                        m_contacts.push_back(cont);
                        found = true;
                        break;
                    }
                }
                if (found) {
                    break;
                }
            }
            if (found) {
                continue;
            }

            // take single edgeedge if possible
            if (temp_contacts[0].size() > 0 &&
                temp_contacts[0].size() < temp_contacts[1].size()) {
                for (int i = 0; i < temp_contacts[0].size(); i++) {
                    m_contacts.push_back(temp_contacts[0][i]);
                }
            } else if (temp_contacts[1].size() > 0 &&
                        temp_contacts[0].size() >
                            temp_contacts[1].size()) {
                for (int i = 0; i < temp_contacts[1].size(); i++) {
                    m_contacts.push_back(temp_contacts[1][i]);
                }
            } else if (temp_contacts[0].size() > 0) {
                for (int i = 0; i < temp_contacts[0].size(); i++) {
                    m_contacts.push_back(temp_contacts[0][i]);
                }
            } else if (temp_contacts[1].size() > 0) {
                for (int i = 0; i < temp_contacts[1].size(); i++) {
                    m_contacts.push_back(temp_contacts[1][i]);
                }
            }
        }
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
        Eigen::Vector3d va = contact.a->getVelocity(contact.p);
        Eigen::Vector3d vb = contact.b->getVelocity(contact.p);
        Eigen::Vector3d n = -contact.n;
        Eigen::Vector3d ra = contact.p - contact.a->getPosition();
        Eigen::Vector3d rb = contact.p - contact.b->getPosition();
        Eigen::Vector3d ran = ra.cross(n);
        Eigen::Vector3d rbn = rb.cross(n);
        Eigen::Matrix3d iInva = contact.a->getInertiaInvWorld();
        Eigen::Matrix3d iInvb = contact.b->getInertiaInvWorld();


        double vrel = n.dot(va - vb);
        if (vrel > 0) {
            // bodies are moving apart
            continue;
        }

        double numerator = -(1 + eps)*vrel;

        // denominator
        double Ma = contact.a->getMassInv();
        double Mb = contact.b->getMassInv();
        double M = Ma + Mb;


        double termA = n.dot((iInva*ran).cross(ra));
        double termB = n.dot((iInvb*rbn).cross(rb));

        // impulse magnitude
        double j = numerator / (M + termA + termB);
        Eigen::Vector3d force = j * n;

        Eigen::Vector3d v_new_a = contact.a->getLinearMomentum() + force;
        Eigen::Vector3d v_new_b = contact.b->getLinearMomentum() - force;

        Eigen::Vector3d w1 = contact.a->getAngularMomentum() + ra.cross(force);
        Eigen::Vector3d w2 = contact.b->getAngularMomentum() + rb.cross(-force);


        // vrel <= 0 -> bodies move towards each other
        // we have no friction so

        //Different outcomes depending on object type
        switch (contact.a->getType()) {
            case ObjType::DYNAMIC:
                contact.a->setLinearMomentum(v_new_a);
                contact.a->setAngularMomentum(w1);
                break;
            case ObjType::ROTATION_ONLY:
                contact.a->setAngularMomentum(w1);
                break;
            case ObjType::STATIC:
                break;
        }

        switch (contact.b->getType()) {
            case ObjType::DYNAMIC:
                contact.b->setLinearMomentum(v_new_a);
                contact.b->setAngularMomentum(w1);
                break;
            case ObjType::ROTATION_ONLY:
                contact.b->setAngularMomentum(w1);
                break;
            case ObjType::STATIC:
                break;
        }

        //Apply effects stored in RigidObjects eg.
        //play sound effect or increase score
        for(auto&& e : contact.a->getEffects) e.apply();
        for(auto&& e : contact.b->getEffects) e.apply();
    }
}