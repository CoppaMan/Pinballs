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


bool isTableCollision2(Shape &ball, Shape &table, vec3 table_normal, Contact &contact) {
    // ball
    vec3 mb = ball.V.colwise().minCoeff();
    vec3 Mb = ball.V.colwise().maxCoeff();

    vec3 center = (mb + Mb) /2;
    double radius = (ball.V.row(0).transpose() - center).norm();

    vec3 Q = table.V.row(0).transpose();

    double distance_to_plane = fabs((Q - center).dot(table_normal));
    vec3 collisionPoint =  center - table_normal*distance_to_plane;

    contact.n = table_normal;
    contact.p = collisionPoint;
    contact.penetration = radius - distance_to_plane;

    if (distance_to_plane < radius) {
        return true;
    }

    return false;
}

bool CollisionDetection::isTableCollision(std::shared_ptr<RigidObject> obj1, std::shared_ptr<RigidObject> obj2, Contact &contact) {
    if (!obj2->isTable()) {
        return false;
    }
    vec3 table_normal = obj2->getRotationMatrix()*vec3(0,1,0);

    Vertices V1;
    Faces F1;
    obj1->getMesh(V1, F1);
    Shape A(V1, F1);

    Vertices V2;
    Faces F2;
    obj2->getMesh(V2, F2);
    Shape B(V2, F2);

    Vertices Vroof;
    Faces Froof;
    vec3 old_pos = obj2->getPosition();
    obj2->setPosition(old_pos + Eigen::Vector3d(0, 0.7, 0));
    obj2->getMesh(Vroof, Froof);
    Shape R(Vroof, Froof);
    obj2->setPosition(old_pos);

    vec3 roof_normal = -table_normal;
    bool roof_collision = isTableCollision2(A, R, roof_normal, contact);
    if (roof_collision) {
        std::cout << "roof collision" << std::endl;
        return true;
    }

    return isTableCollision2(A, B, table_normal, contact);

}

bool isTableContinuous(std::shared_ptr<RigidObject> obj1, std::shared_ptr<RigidObject> obj2, Contact &contact) {
    double tmin = 0;
    double tmax = 1;

    double best_t = 2;
    while (tmin <= tmax) {

    }
}

void CollisionDetection::computeNarrowPhase(int narrowPhaseMethod, float &timeDelta) {
    switch (narrowPhaseMethod) {
        case 0: {
        
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

                bool isnotdynamic_1 = obj1->getType() != ObjType ::DYNAMIC;
                bool isnotdynamic_2 = obj2->getType() != ObjType ::DYNAMIC;

                if (isnotdynamic_1 && isnotdynamic_2)
                    continue;

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

                if (obj2->isTable()) {
                    if (isTableCollision(obj1, obj2, contact)) {
                        m_contacts.push_back(contact);
                    }
                    continue;
                }
                
                bool CCD = true;

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

        bool isNotDynamicA = contact.a->getType() != ObjType::DYNAMIC;
        bool isNotDynamicB = contact.a->getType() != ObjType::DYNAMIC;
        
        if (isNotDynamicA && isNotDynamicB)
            continue;

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

        vec3 tangent = (contact.n.cross(vrel_vec).cross(contact.n)).normalized();

        if(contact.a->getType() == ObjType::ROTATION_ONLY || contact.a->getType() == ObjType::ROTATION_ONLY) {
            force *= 2;
        } else {
            force *= 0.8;
        }

        if (contact.a->getType() == ObjType::DYNAMIC) {
            float mu = 0.42;
            vec3 friction = mu*j.dot(-contact.n)*tangent;
            contact.a->setLinearMomentum(contact.a->getLinearMomentum() + force);
            contact.a->setAngularMomentum(contact.a->getAngularMomentum() +  ra.cross(force));
            
            float e = 0.9;
            float d = contact.a->getLinearMomentum().dot(contact.n);
            float k = - ( 1 + e ) * d;
            float j2 = k < 0 ? 0 : k;
            contact.a->setLinearMomentum(contact.a->getLinearMomentum() + j2*contact.n);
        }

        if (contact.b->getType() == ObjType::DYNAMIC) {
            float mu = 0.42;
            vec3 friction = mu*j.dot(contact.n)*tangent;
            contact.b->setLinearMomentum(contact.b->getLinearMomentum() - force);
            contact.b->setAngularMomentum(contact.b->getAngularMomentum() -  rb.cross(force));
        }

        if (debug) {
            contact.a->printDebug("After update");
            contact.b->printDebug("After update");
        }

        //Apply effects stored in RigidObjects eg.
        //play sound effect or increase score
        for(auto e : contact.a->getEffects()) e->apply(contact.b);
        for(auto e : contact.b->getEffects()) e->apply(contact.a);
    }
}