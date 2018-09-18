#ifndef BASEOBJECT_H
#define BASEOBJECT_H

#include <igl/per_face_normals.h>
#include <igl/per_vertex_normals.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <Eigen/Core>

struct Mesh {
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    // Per face attributes
    Eigen::MatrixXd F_normals;  // One normal per face

    // Per vertex attributes
    Eigen::MatrixXd V_normals;  // One normal per vertex

    // UV parametrization
    Eigen::MatrixXd V_uv;  // UV vertices
    Eigen::MatrixXi F_uv;  // optional faces for UVs
};

class BaseObject {
   public:
    bool loadMesh(const std::string& path);

    void findAndLoadMesh(const std::string& file);

    void reset();

    void setScale(double s);
    void setID(int id);
    void setPosition(const Eigen::Vector3d& p);
    void setRotation(const Eigen::Quaterniond& q);
    void setRotation(const Eigen::Matrix3d& R);

    double getScale();
    int getID();
    Eigen::Vector3d getPosition();
    Eigen::Quaterniond getRotation();
    Eigen::Matrix3d getRotationMatrix();

   protected:
    /*
     * Reset class variables specific to a certain object. Is called by
     * BaseObject::reset().
     */
    virtual void resetMembers() = 0;

    int m_id = -1;
    Mesh m_mesh;

    double m_scale = 1.0;        // Scale
    Eigen::Vector3d m_position;  // Position of the center of mass
    Eigen::Quaterniond m_quat;   // Rotation (quaternion)
    Eigen::Matrix3d m_rot;       // Rotation (matrix)
};

#endif