#include "CollisionDetection.h"
#include "BoundingSphere.h"
#include "CollisionDetectionHelper.h"
#include "SAT.h"
#include "gjk2.h"
#include <algorithm> 


inline bool isCollision(std::shared_ptr<BoundingObject> r1, std::shared_ptr<BoundingObject> r2) {
    switch (r1->getBoundingType()) {
        case SPHERE: {
            std::shared_ptr<BoundingSphere> s1 = std::dynamic_pointer_cast<BoundingSphere> (r1);
            switch (r2->getBoundingType()) {
                case SPHERE:
                    return CollisionDetectionHelper::isCollision(s1, std::dynamic_pointer_cast<BoundingSphere>(r2));
                case BOX:
                    return CollisionDetectionHelper::isCollision(s1, std::dynamic_pointer_cast<BoundingBox> (r2));
            }
        }
        case BOX: {
            std::shared_ptr<BoundingBox> b1 = std::dynamic_pointer_cast<BoundingBox> (r1);
            switch (r2->getBoundingType()) {
                case SPHERE:
                    return CollisionDetectionHelper::isCollision(std::dynamic_pointer_cast<BoundingSphere>(r2), b1);
                case BOX:
                    return CollisionDetectionHelper::isCollision(b1, std::dynamic_pointer_cast<BoundingBox> (r2));
            }
        }
    }
    throw "Should never reach this point";
}

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
        std::vector<std::shared_ptr<BoundingObject>> aabbs(m_objects.size());
        for (size_t i = 0; i < aabbs.size(); i++) {
            aabbs[i] = m_objects[i]->getBoundingObject();
        }


        for (size_t i = 0; i < m_objects.size(); i++) {
            for (size_t j = i + 1; j < m_objects.size(); j++) {
                // add pair of objects to possible collision if their
                // bounding boxes overlap
                bool a_dynamic = m_objects[i]->getType() == ObjType ::DYNAMIC;
                bool b_dynamic = m_objects[j]->getType() == ObjType ::DYNAMIC;
                if ((a_dynamic || b_dynamic) && isCollision(aabbs[i], aabbs[j])) {
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



void CollisionDetection::computeNarrowPhase(int narrowPhaseMethod, double timeDelta) {
    switch (narrowPhaseMethod) {
        case 0: {
            // exhaustive
            // iterate through all pairs of possible collisions
            for (auto overlap : m_overlappingBodys) {
                std::vector<Contact> temp_contacts[2];
                // compute intersection of a with b first and intersectino
                // of b with a and store results in temp_contacts
                for (int switcher = 0; switcher < 2; switcher++) {
                    auto a =
                        m_objects[(!switcher) ? overlap.first
                                                : overlap.second];
                    auto b =
                        m_objects[(!switcher) ? overlap.second
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

            for (auto overlap : m_overlappingBodys) {
                auto obj1 = m_objects[overlap.first];
                auto obj2 = m_objects[overlap.second];

                Vertices V1;
                Faces F1;
                obj1->getMesh(V1, F1);

                Vertices V2;
                Faces F2;
                obj2->getMesh(V2, F2);

                Contact contact;
                contact.a = obj1;
                contact.b = obj2;

                if (SAT::intersect(V1, F1, V2, F2, contact)) {
                    m_contacts.push_back(contact);
                }
            }
            // TODO
            break;
        } 
        case 2: {
            for (auto overlap: m_overlappingBodys) {
                auto obj1 = m_objects[overlap.first];
                auto obj2 = m_objects[overlap.second];

                Vertices V1;
                Faces F1;
                obj1->getMesh(V1, F1);
                Shape A(V1, F1);

                Vertices V2;
                Faces F2;
                obj2->getMesh(V2, F2);
                Shape B(V2, F2);

                Contact contact;
                contact.a = obj1;
                contact.b = obj2;
                
                bool CCD = false;

                if(CCD) {
                    if (GJK::runWithCCD(A, obj1->getLinearVelocity()*timeDelta, B, obj2->getLinearVelocity()*timeDelta, contact)) {
                        m_contacts.push_back(contact);
                    }
                } else {
                    if (GJK::run(A, B, contact)) {
                        m_contacts.push_back(contact);
                    }
                }  
            }
        }
    }
}

void CollisionDetection::applyImpulse(double eps) {
    // compute impulse for all contacts
    for (auto contact : m_contacts) {
        if(debug) {
            contact.a->printDebug("Before update");
            contact.b->printDebug("Before update");
            std::cout << "Contact normal:" << std::endl;
            std::cout << contact.n << std::endl;
        }
        Eigen::Vector3d vrel_vec = contact.a->getVelocity(contact.p) -
                                   contact.b->getVelocity(contact.p);
        double vrel = contact.n.dot(vrel_vec);
        if (vrel > 0) {
            // bodies are moving apart
            continue;
        }

        // TODO: compute impulse and update the following momentums
        Eigen::Vector3d numerator = -(1 + eps) * vrel_vec;
        double t1 = contact.a->getMassInv();
        double t2 = contact.b->getMassInv();
        Eigen::Vector3d ra = contact.p - contact.a->getPosition();
        Eigen::Vector3d rb = contact.p - contact.b->getPosition();
        double t3 = contact.n.dot(
                (contact.a->getInertiaInvWorld() * (ra.cross(contact.n)))
                        .cross(ra));
        double t4 = contact.n.dot(
                (contact.b->getInertiaInvWorld() * (rb.cross(contact.n)))
                        .cross(rb));

        Eigen::Vector3d j = numerator / (t1 + t2 + t3 + t4);
        Eigen::Vector3d force = (j.dot(contact.n) * contact.n);

        bool isDynamicA = contact.a->getType() == ObjType::DYNAMIC;
        bool isDynamicB = contact.a->getType() == ObjType::DYNAMIC;

        if (contact.a->getType() == ObjType::DYNAMIC) {
            contact.a->setLinearMomentum(contact.a->getLinearMomentum() + force);
            contact.a->setAngularMomentum(contact.a->getAngularMomentum() +  ra.cross(force));
        }

        if (contact.b->getType() == ObjType::DYNAMIC) {
            contact.b->setLinearMomentum(contact.b->getLinearMomentum() - force);
            contact.b->setAngularMomentum(contact.b->getAngularMomentum() -  rb.cross(force));
        }

        if (debug) {
            contact.a->printDebug("After update");
            contact.b->printDebug("After update");
        }
        //Apply effects stored in RigidObjects eg.
        //play sound effect or increase score
        for(auto e : contact.a->getEffects()) e->apply();
        for(auto e : contact.b->getEffects()) e->apply();
    }
}