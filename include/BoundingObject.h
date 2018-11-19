#ifndef BOUNDINGOBJECT
#define BOUNDINGOBJECT


enum BOUNDING_TYPE {
    SPHERE, BOX
};

class BoundingObject {

public:
    BoundingObject(BOUNDING_TYPE type) : m_bounding_type(type) {}

    virtual ~BoundingObject() = default; // do not remove this is to make BoundingObject abstract class

    BOUNDING_TYPE  getBoundingType() {
        return m_bounding_type;
    }

private:
    BOUNDING_TYPE m_bounding_type;
};

#endif // BOUNDINGOBJECT