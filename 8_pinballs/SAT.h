#ifndef H_SAT
#define H_SAT

#include <Eigen/Core>
#include <igl/per_face_normals.h>
#include "CollisionDetection.h"


typedef Eigen::Vector3d vec3;
typedef Eigen::MatrixXd Vertices;
typedef Eigen::MatrixXi Faces;

struct Projection {
    double min;
    double max;
};

struct Axis {
    vec3 n;
    int p1;
    int p2;

};

class SAT {

public:

    static bool check_overlap(std::vector<Axis> &axis, Vertices &V1, Vertices &V2, Contact &contact) {
        for (Axis ax : axis) {
            Projection p1 = project(V1, ax.n);
            Projection p2 = project(V2, ax.n);

            if (!overlap(p1, p2)) {
                return false;
            }
            contact.n = ax.n;
            contact.p = V1.row(ax.p1).transpose(); // just take one of the points
        }
        return true;
    }
    static bool intersect(Vertices &V1, Faces &F1, Vertices &V2, Faces &F2, Contact &contact) {
        std::vector<Axis> axis1 = getAxis(V1, F1);
        std::vector<Axis> axis2 = getAxis(V2, F2);

        if (!check_overlap(axis1, V1, V2, contact) || !check_overlap(axis2, V2, V1, contact) ) {
            return false;
        }
        return true;
    }

    static bool overlap(Projection p1, Projection p2) {
        return !(p1.max < p2.min || p2.max < p1.min);
    }

    static Projection project(Vertices &V, vec3 &n) {
        Eigen::VectorXd val = V*n;
        double min = val.minCoeff();
        double max = val.maxCoeff();
        return {min, max};
    }

    static std::vector<Axis> getAxis(Vertices &V, Faces &F) {
        Eigen::MatrixXd N_faces; // Should be N*3

        igl::per_face_normals(V, F, N_faces);

        std::vector<Axis> axis;

        for (int i = 0; i < F.rows(); i++) {
            Eigen::Matrix<int, 1, Eigen::Dynamic> face = F.row(i);
            int p1 = face.col(0).value();
            int p2 = face.col(1).value();
            int p3 = face.col(2).value();

            vec3 face_normal = N_faces.row(i).transpose().normalized();


            vec3 e1 = V.row(p2).transpose() - V.row(p1).transpose();
            vec3 e2 = V.row(p3).transpose() - V.row(p2).transpose();
            vec3 e3 = V.row(p1).transpose() - V.row(p3).transpose();


            vec3 n1 = e1.cross(face_normal).normalized();
            vec3 n2 = e2.cross(face_normal).normalized();
            vec3 n3 = e3.cross(face_normal).normalized();


            Axis ax1 = {n1, p1, p2};
            Axis ax2 = {n2, p1, p2};
            Axis ax3 = {n3, p1, p2};
            axis.push_back(ax1);
            axis.push_back(ax2);
            axis.push_back(ax3);
        }

        //igl::per_edge_normals(V, F, igl::PER_EDGE_NORMALS_WEIGHTING_TYPE_UNIFORM, )
        return axis;
    }

};

#endif