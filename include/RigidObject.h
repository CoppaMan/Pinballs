#ifndef RIGIDOBJECT_H
#define RIGIDOBJECT_H

#include "BaseObject.h"
#include "../8_pinballs/Effect.h"
#include "BoundingObject.h"

class Effect;

/*
 * Base class representing a simple rigid object.
 */
class RigidObject : public BaseObject {
   public:
    RigidObject();
    RigidObject(const std::string& mesh_path,
                const ObjType t = ObjType::DYNAMIC,
                BOUNDING_TYPE bounding_Type = BOUNDING_TYPE::BOX,
                bool isTable = false);

    void applyForceToCOM(const Eigen::Vector3d& f);
    /*
     * Apply force f at point p.
     */
    void applyForce(const Eigen::Vector3d& f, const Eigen::Vector3d& p);
    void applyTorque(const Eigen::Vector3d& t);
    void printDebug(const std::string& message = "") const;

    void addEffect(std::shared_ptr<Effect> e);
    std::vector<std::shared_ptr<Effect>> getEffects();

#pragma region GettersAndSetters
    virtual void setType(ObjType t);
    void setMass(double m);
    void setInertia(const Eigen::Matrix3d& I);
    void setLinearMomentum(const Eigen::Vector3d& p);
    void setAngularMomentum(const Eigen::Vector3d& l);
    void setLinearVelocity(const Eigen::Vector3d& v);
    void setAngularVelocity(const Eigen::Vector3d& w);
    void setForce(const Eigen::Vector3d& f);
    void setTorque(const Eigen::Vector3d& t);
    void resetForce();
    void resetTorque();

    virtual bool isTable() {
        return m_istable;
    }

    virtual bool renderObject() {
        return true;
    }

    virtual std::shared_ptr<BoundingObject> getBoundingObject() const;
    BOUNDING_TYPE getBoundingType() const;
    double getMass() const;
    double getMassInv() const;
    Eigen::Matrix3d getInertia() const;
    Eigen::Matrix3d getInertiaInv() const;
    Eigen::Matrix3d getInertiaInvWorld() const;
    Eigen::Matrix3d getInertiaWorld() const;
    Eigen::Vector3d getLinearMomentum() const;
    Eigen::Vector3d getAngularMomentum() const;
    virtual Eigen::Vector3d getLinearVelocity() const;
    Eigen::Vector3d getVelocity(const Eigen::Vector3d& point) const;
    Eigen::Vector3d getAngularVelocity() const;
    virtual Eigen::Vector3d getForce() const;
    Eigen::Vector3d getTorque() const;
#pragma endregion GettersAndSetters

   protected:
    virtual void resetMembers() override;
    bool m_istable;
   private:
    double m_mass;                 // Body mass
    double m_massInv;              // Inverted mass
    BOUNDING_TYPE m_bounding_type;
    Eigen::Matrix3d m_inertia;     // Intertial Tensor (initially set to cube)
    Eigen::Matrix3d m_inertiaInv;  // Inverse

    Eigen::Vector3d m_v;  // Linear velocity
    Eigen::Vector3d m_w;  // Angular velocity

    Eigen::Vector3d m_force;   // Force on body
    Eigen::Vector3d m_torque;  // Torque on body

    std::vector<std::shared_ptr<Effect>> effects; // Triggers certain effects during different parts of the simulation

};

#endif