#include <RigidObject.h>

class Paddle : public RigidObject {
public:
    Paddle() : RigidObject("flipper.off", ObjType::STATIC) {

    }
};