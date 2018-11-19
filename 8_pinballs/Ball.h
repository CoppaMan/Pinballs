#ifndef DEFBALL
#define DEFBALL

#include <RigidObject.h>
#include "BoundingSphere.h"

class Ball : public RigidObject {
public:
    Ball() : RigidObject("sphere.off", ObjType::DYNAMIC, BOUNDING_TYPE::SPHERE) {

    }

    void advance() {

    }

    std::shared_ptr<BoundingObject> getBoundingObject() const override {
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        getMesh(V, F);
        return std::make_shared<BoundingSphere>(V);
    }

};


#endif // DEFBALL