#ifndef H_SAT
#define H_SAT

#include <Eigen/Core>
#include <igl/per_face_normals.h>
#include "CollisionDetection.h"



struct Projection {
    double min;
    double max;
};

struct Axis {
    vec3 n;
    int p1;
    int p2;

};

struct SATResult {
    vec3 s_min;
    float d_min;
};


class SAT {

public:


    static vec3 proj(vec3 &v, vec3 &e, vec3 &v0_e) {
        return v0_e + e*((v - v0_e)).dot(e);
    }

    static SATResult run(Shape &A, Shape &B, Contact &contact) {
        SATResult result = {vec3(0,0,0), -10000};


        for (int i = 0; i < A.F.rows(); i++) { // check FaceVertex
            vec3 n = -A.N_faces.row(i).transpose();
            checkDirections(n, A, B, result);
        }
        for (int i = 0; i < B.F.rows(); i++) { // check VertexFace
            vec3 n = B.N_faces.row(i).transpose();
            checkDirections(n, A, B, result);
        }

        for (int i = 0; i < A.F.rows(); i++) { // EE -> this is the n^2 part which we should optimize
            for (int j = 0; j < B.F.rows(); j++) {
                for (int k = 0; k < 2; k++) {
                    int Ap1 = A.F.row(i).col(k).value();
                    int Ap2 = A.F.row(i).col((k + 1) % 3).value();

                    int Bp1 = B.F.row(i).col(k).value();
                    int Bp2 = B.F.row(i).col((k + 1) % 3).value();

                    vec3 Aedge = (A.V.row(Ap2).transpose() - A.V.row(Ap1).transpose()).normalized();
                    vec3 Bedge = (B.V.row(Bp2).transpose() - B.V.row(Bp1).transpose()).normalized();

                    vec3 s_new = Aedge.cross(Bedge);
                    checkDirections(s_new, A, B, result);
                    checkDirections(-s_new, A, B, result);
                }
            }
        }

        if (result.d_min <= 0) {
            return result; // skip VV, VE, EV
        }

        // VV
        for (int i = 0; i < A.V.rows(); i++) { 
            vec3 vA = A.V.row(i).transpose();
            for (int j = 0; j < B.V.rows(); j++) {
                vec3 vB = B.V.row(j).transpose();
                checkDirections(vA - vB, A, B, result);
            }
        }

        // VE
        for (int i = 0; i < A.V.rows(); i++) { 
            vec3 vA = A.V.row(i).transpose();
            for (int j = 0; j < B.F.rows(); j++) {
                for (int k = 0; k < 2; k++) {
                    int Bp1 = B.F.row(j).col(k).value();
                    int Bp2 = B.F.row(j).col((k + 1) % 3).value();
                    vec3 e_v1 = B.V.row(Bp1).transpose();
                    vec3 Bedge = (B.V.row(Bp2).transpose() - e_v1).normalized();
                    // possible optimization: proj(vA, edgeB) element of edgeB only then check 
                    checkDirections(vA - proj(vA, Bedge, e_v1), A, B, result);
                }
            }
        }

        // EV
        for (int i = 0; i < B.V.rows(); i++) { 
            vec3 vB = B.V.row(i).transpose();
            for (int j = 0; j < A.F.rows(); j++) {
                for (int k = 0; k < 2; k++) {
                    int Ap1 = A.F.row(j).col(k).value();
                    int Ap2 = A.F.row(j).col((k + 1) % 3).value();
                    vec3 e_v1 = A.V.row(Ap1).transpose();
                    vec3 Aedge = (A.V.row(Ap2).transpose() - e_v1).normalized();
                    // possible optimization: proj(vA, edgeB) element of edgeB only then check 
                    checkDirections(vB - proj(vB, Aedge, e_v1), A, B, result);
                }
            }
        }

        return result;

    }


    inline static bool checkDirections(const vec3 &s, const Shape &A, const Shape &B, SATResult &results) {
        if (s.norm() != 0) {// ignore degenerate case
            vec3 s_ = s.normalized();
            double d = support(A, s_) + support(B, -s_);
            if (d > results.d_min) {
                results.d_min = d;
                results.s_min = s_;
                return true;
            }
        }
        return false;
    }

    inline static double support(const Shape &A, const vec3 &s) {
        Eigen::VectorXd proj = A.V*s;
        //Eigen::MatrixXf::Index maxIndex;
        //proj.maxCoeff(&maxIndex);
        //return A.V.row(maxIndex).transpose();
        return proj.minCoeff();
    }


    static bool check_overlap(std::vector<Axis> &axis, Vertices &V1, Vertices &V2, Contact &contact) {
        for (Axis ax : axis) {
            Projection p1 = project(V1, ax.n);
            Projection p2 = project(V2, ax.n);

            auto objA = contact.a;
            auto objB = contact.b;


            if (!overlap(p1, p2)) {
                return false;
            }
            contact.n = ax.n;
            contact.p = V1.row(ax.p1).transpose(); // just take one of the points
        }
        return true;
    }

    static bool intersect(Vertices &V1, Faces &F1, Vertices &V2, Faces &F2, Contact &contact) {
        Shape A(V1, F1);
        Shape B(V2, F2);

        SATResult result =  run(A, B, contact);
        return result.d_min <= 0;
    }

    static bool intersect2(Vertices &V1, Faces &F1, Vertices &V2, Faces &F2, Contact &contact) {
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