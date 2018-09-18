#ifndef RIGIDOBJECT_H
#define RIGIDOBJECT_H

#include "BaseObject.h"

/*
 * Base class representing a simple rigid object.
 */
class RigidObject : public BaseObject {
   public:
    RigidObject() {}
    RigidObject(const std::string& mesh_path);

    void applyForceToCOM(const Eigen::Vector3d& f);
    /*
     * Apply force f at point p.
     */
    void applyForce(const Eigen::Vector3d& f, const Eigen::Vector3d& p);
    void applyTorque(const Eigen::Vector3d& t);

#pragma region GettersAndSetters
    void setMass(double m);
    void setInertia(const Eigen::Matrix3d& I);
    void setLinearMomentum(const Eigen::Vector3d& p);
    void setAngularMomentum(const Eigen::Vector3d& l);
    void setLinearVelocity(const Eigen::Vector3d& v);
    void setForce(const Eigen::Vector3d& f);
    void setTorque(const Eigen::Vector3d& t);
    void resetForce();
    void resetTorque();

    double getMass();
    double getMassInv();
    Eigen::Matrix3d getInertia();
    Eigen::Matrix3d getInertiaInv();
    Eigen::Matrix3d getInertiaInvWorld();
    Eigen::Vector3d getLinearMomentum();
    Eigen::Vector3d getAngularMomentum();
    Eigen::Vector3d getLinearVelocity();
    Eigen::Vector3d getAngularVelocity();
    Eigen::Vector3d getForce();
    Eigen::Vector3d getTorque();
    void getMesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F);
#pragma endregion GettersAndSetters

   protected:
    virtual void resetMembers() override;

   private:
    double m_mass;                 // Body mass
    double m_massInv;              // Inverted mass
    Eigen::Matrix3d m_inertia;     // Intertial Tensor (initially set to cube)
    Eigen::Matrix3d m_inertiaInv;  // Inverse

    Eigen::Vector3d m_p;  // Linear momentum
    Eigen::Vector3d m_l;  // Angular momentum

    Eigen::Vector3d m_force;   // Force on body
    Eigen::Vector3d m_torque;  // Torque on body
};

#endif