#ifndef GJK_HEADER
#define GJK_HEADER

#include <Eigen/Core>
#include <igl/per_face_normals.h>
#include <vector>
#include "CollisionDetection.h"


double EPSILON = 1e-4;

class GJK {
public:
    GJK();
    ~GJK();


    static bool run(const Shape &A, const Shape &B, Contact &contact) {
        int GJK_MAXITR = 32;
        vec3 new_search_dir = A.V.row(0).transpose() - B.V.row(0).transpose();
        vec3 a,b,c,d;
        int num_dim = 0;
        a = support(new_search_dir, A, B); // support in direction of origin
        num_dim++;

        // a squared is distance to origin
        // a.dot(new_search_dir) is equal to a*a if new_search_dir has not changed
        for (int iters = 0; iters < GJK_MAXITR; iters++) {
            // closest point to simplex should make sure that a is free to set it
            // with value of new search direction
            if (closest_point_to_simplex(a,b,c,d, new_search_dir,  num_dim)) {
                // collision
                vec3 normal;
                float penetration;
                vec3 p;
                if (EPA(a, b, c, d, A, B, normal, penetration, p)) {
                    contact.p = p;
                    contact.n = normal;
                    contact.penetration = penetration;
                    std::cout << "Collision" << std::endl;
                    return true;
                } else {
                    return false;
                }
            }
            a = support(new_search_dir, A, B);

            if (a.dot(new_search_dir) < 0) { // -new_search_dir is seperating axis
                return false;
            }
        }
        return false;
    }

    static bool runWithCCD(const Shape &A, const Shape &B, Contact &contact, double timeDelta) {
        int CCD_STEPS = 32;
        int GJK_MAXITR = 32;
        vec3 a_b, b_b, c_b, d_b;
        vec3 vel_a = contact.a->getLinearVelocity()*timeDelta;
        vec3 vel_b = contact.b->getLinearVelocity()*timeDelta;
        vec3 rot_a = contact.a->getAngularVelocity()*timeDelta;
        vec3 rot_b = contact.b->getAngularVelocity()*timeDelta;
        Eigen::MatrixXd A_b(A.V.rows(), A.V.cols());
        Eigen::MatrixXd B_b(B.V.rows(), B.V.cols());
        double ratio = 1; // Fraction of velocity at which both objects touch the first time
        double factor = 1;
        double dir;

        for(int steps = 0; steps < CCD_STEPS; steps++) {

            // 1. Check for collision
            vec3 new_search_dir = A.V.row(0).transpose() - B.V.row(0).transpose();
            vec3 a,b,c,d;
            int num_dim = 0;
            a = support(new_search_dir, A, B); // support in direction of origin
            num_dim++;
            bool does_collide = false;

            for (int iters = 0; iters < GJK_MAXITR; iters++) {
                if (closest_point_to_simplex(a,b,c,d, new_search_dir,  num_dim)) {
                    does_collide = true;
                }
                a = support(new_search_dir, A, B);
                if (a.dot(new_search_dir) < 0) break;
            }

            if(!does_collide && steps == 0) {
                std::cout << "No initial collision" << std::endl;
                return false; // Stop if original position doesn't collide
            }

            // 2. Check position
            std::cout << "CCD pass " << steps + 1 << " Ratio: " << ratio << " => ";
            if(does_collide) { // Move and rotate half a step back (still colliding)
                std::cout << "Collision" << std::endl;
                a_b = a; b_b = b; c_b = c; d_b = d;
                A_b = A.V; B_b = B.V;
                dir = -1;
                contact.ratio = ratio;
            } else {
                std::cout << "No collision" << std::endl;
                dir = 1;
            }

            // 3. Correction
            if(steps < CCD_STEPS-1) {
                factor /= 2;
                ratio = ratio + dir*factor;
                A.V = A.V.rowwise() + dir*vel_a.transpose()*factor;
                B.V = B.V.rowwise() + dir*vel_b.transpose()*factor;
            }
        }

        // 4. Find collision parameters
        vec3 normal;
        float penetration;
        vec3 p;
        A.V = A_b; B.V = B_b;
        if (EPA(a_b, b_b, c_b, d_b, A, B, normal, penetration, p)) { //EPA finds collision data
            contact.p = p;
            contact.n = normal;
            contact.penetration = penetration;
            return true;
        }
        return false;
    }



    static vec3 handle_line(vec3 &a, vec3 &b, vec3 &c, vec3 &d, int &num_dim) {
        vec3 ab = b - a;
        vec3 ao = -a;
        vec3 new_search_dir;
        if (ao.dot(ab) > 0) {
            // [a,b]
            num_dim = 3; // origin inbweween ab
            new_search_dir = ab.cross(ao).cross(ab);
            c = b; b = a;

        } else {
            // [a]
            num_dim = 2; // origin on side a
            new_search_dir = ao; // in direction of origin
            b = a;
        }
        return new_search_dir;
    }

    static vec3 handle_triangle(vec3 &a, vec3 &b, vec3 &c, vec3 &d, int &num_dim) {
        vec3 ab = b - a;
        vec3 ac = c - a;
        vec3 ao = - a; // 0 -a
        vec3 abc = ab.cross(ac);

        vec3 new_search_dir;

        if ((abc.cross(ac)).dot(ao) > 0) {
            if (ac.dot(ao) > 0) {
                // [a, c]
                new_search_dir = ac.cross(ao).cross(ac);
                num_dim = 3;
                b = a;
            } else {
                //****
                if (ab.dot(ao) > 0) {
                    // [a,b]
                    new_search_dir = ab.cross(ao).cross(ab);
                    c = b; b = a;
                    num_dim = 3;
                } else {
                    // [a]
                    new_search_dir = ao;
                    b = a;
                    num_dim = 2;
                }
                // ****
            }
        } else {
            if ((ab.cross(abc)).dot(ao) > 0) {
                //****
                if (ab.dot(ao) > 0) {
                    // [a,b]
                    new_search_dir = ab.cross(ao).cross(ab);
                    num_dim = 3;
                    c = b; b = a;
                } else {
                    // [a]
                    new_search_dir = ao;
                    num_dim = 2;
                    b = a;
                }
                // ****
            } else {
                if (abc.dot(ao) > 0) {
                    // [a,b,c]
                    new_search_dir = abc;
                    num_dim = 4;
                    d = c; c = b; b = a;
                } else {
                    // -[a,b,c]
                    new_search_dir = -abc;
                    num_dim = 4;
                    d = c; c = b; b = a;
                }
            }
        }
        return new_search_dir;
    }

    static bool handle_thetrahidral(vec3 &a, vec3 &b, vec3 &c, vec3 &d, vec3 &new_search_dir, int &num_dim) {
        vec3 ABC = (c-a).cross(b-a);
        vec3 ACD = (c-a).cross(d-a);
        vec3 ADB = (d-a).cross(b-a);

        vec3 AO = -a;
        num_dim = 4;

        if (ABC.dot(AO) > 0) {
            d = c; c = b; b = a;
            new_search_dir = ABC;
        }
        else if (ACD.dot(AO) > 0) {
            b = a;
            new_search_dir = ACD;
        }
        else if (ADB.dot(AO) > 0) {
            c = d; d = b; b = a;
            new_search_dir = ADB;
        } else {
            // if we reach this point we have enclosed the origin and
            // a collision has occured!!
            return true;
        }

        return false;
    }

    static bool closest_point_to_simplex(vec3 &a, vec3 &b, vec3 &c, vec3 &d, vec3 &new_search_dir, int &num_dim) {
        // Johnsons algorithm
        if (num_dim == 1) {
            num_dim = 2;
            b = a;
            new_search_dir = -a;  // 1 point only 1 point can be nearest so we search in direction of origin
            return false;
        }
        if (num_dim == 2) { // line segment
            new_search_dir = handle_line(a,b,c,d, num_dim);
            return false;
        } else if (num_dim == 3) { // we're triangle
            new_search_dir = handle_triangle(a, b, c, d, num_dim);
            return false;
        } else { // num_dim == 4
            if (handle_thetrahidral(a, b, c, d, new_search_dir, num_dim)) {
                return true; // collision
            }
        }
        return false;
    }


    static vec3 support(const vec3 &v, const Shape &A, const Shape &B) {
        return support(v, A) - support(-v, B);
    }

    static vec3 support(const vec3 &v, const Shape &S) {
        Eigen::VectorXd result = S.V*v;
        Eigen::MatrixXf::Index   maxIndex;
        result.maxCoeff(&maxIndex);
        return S.V.row(maxIndex).transpose();
    }

    static Eigen::MatrixXf::Index arg_support(const vec3 &v, const Shape &S) {
        Eigen::VectorXd result = S.V*v;
        Eigen::MatrixXf::Index   maxIndex;
        result.maxCoeff(&maxIndex);
        return maxIndex;
    }



    static bool EPA(vec3 &a, vec3 &b, vec3 &c, vec3 &d, const Shape &A, const Shape &B, vec3 &normal, float &penetration, vec3 &contact_point){
        const int EPA_MAX_NUM_FACES = 64;
        const int EPA_MAX_NUM_LOOSE_EDGES = 32;
        const int EPA_MAX_NUM_ITERATIONS = 64;

        vec3 faces[EPA_MAX_NUM_FACES][4]; //Array of faces, each with 3 verts and a normal

        //Init with final simplex from GJK

        faces[0][0] = a; faces[0][1] = b; faces[0][2] = c;  faces[0][3] = (b-a).cross(c-a).normalized(); //ABC
        faces[1][0] = a; faces[1][1] = c; faces[1][2] = d;  faces[1][3] = (c-a).cross(d-a).normalized(); //ACD
        faces[2][0] = a; faces[2][1] = d; faces[2][2] = b;  faces[2][3] = (d-a).cross(b-a).normalized(); //ADB
        faces[3][0] = b; faces[3][1] = d; faces[3][2] = c;  faces[3][3] = (d-b).cross(c-b).normalized(); //BDC

        int num_faces = 4;
        int closest_face;

        for(int iterations=0; iterations<EPA_MAX_NUM_ITERATIONS; iterations++){
            //Find face that's closest to origin
            float min_dist = faces[0][0].dot(faces[0][3]);
            closest_face = 0;
            for(int i=1; i<num_faces; i++){
                float dist = faces[i][0].dot(faces[i][3]);
                if(dist < min_dist) {
                    min_dist = dist;
                    closest_face = i;
                }
            }

            //search normal to face that's closest to origin
            vec3 search_dir = faces[closest_face][3];
            vec3 p = support(search_dir, A, B);

            if(p.dot(search_dir) - min_dist < EPSILON) {
                //Convergence (new point is not significantly further from origin)
                normal = -faces[closest_face][3];
                penetration = p.dot(search_dir);

                vec3 mtv = faces[closest_face][3]*p.dot(search_dir); //dot vertex with normal to resolve collision along normal!
                vec3 arg_p = support(search_dir, A);

                double test = (search_dir - arg_p).norm();
                contact_point = arg_p; // arg_p should always be a point on our mesh //faces[closest_face][2]; // vertex + penetration*normal
                return true;
            }

            vec3 loose_edges[EPA_MAX_NUM_LOOSE_EDGES][2]; //keep track of edges we need to fix after removing faces
            int num_loose_edges = 0;

            //Find all triangles that are facing p
            for(int i=0; i<num_faces; i++)
            {
                if(faces[i][3].dot(p-faces[i][0]) > 0) //triangle i faces p, remove it
                {
                    //Add removed triangle's edges to loose edge list.
                    //If it's already there, remove it (both triangles it belonged to are gone)
                    for(int j=0; j<3; j++) //Three edges per face
                    {
                        vec3 current_edge[2] = {faces[i][j], faces[i][(j + 1) % 3]};
                        bool found_edge = false;
                        for(int k = 0; k < num_loose_edges; k++) //Check if current edge is already in list
                        {
                            if(loose_edges[k][1] == current_edge[0] && loose_edges[k][0] == current_edge[1]){
                                //Edge is already in the list, remove it
                                //THIS ASSUMES EDGE CAN ONLY BE SHARED BY 2 TRIANGLES (which should be true)
                                //THIS ALSO ASSUMES SHARED EDGE WILL BE REVERSED IN THE TRIANGLES (which
                                //should be true provided every triangle is wound CCW)
                                loose_edges[k][0] = loose_edges[num_loose_edges-1][0]; //Overwrite current edge
                                loose_edges[k][1] = loose_edges[num_loose_edges-1][1]; //with last edge in list
                                num_loose_edges--;
                                found_edge = true;
                                k=num_loose_edges; //exit loop because edge can only be shared once
                            }
                        }//endfor loose_edges

                        if(!found_edge){ //add current edge to list
                            // assert(num_loose_edges<EPA_MAX_NUM_LOOSE_EDGES);
                            if(num_loose_edges>=EPA_MAX_NUM_LOOSE_EDGES) break;
                            loose_edges[num_loose_edges][0] = current_edge[0];
                            loose_edges[num_loose_edges][1] = current_edge[1];
                            num_loose_edges++;
                        }
                    }

                    //Remove triangle i from list
                    faces[i][0] = faces[num_faces-1][0];
                    faces[i][1] = faces[num_faces-1][1];
                    faces[i][2] = faces[num_faces-1][2];
                    faces[i][3] = faces[num_faces-1][3];
                    num_faces--;
                    i--;
                }//endif p can see triangle i
            }//endfor num_faces

            //Reconstruct polytope with p added
            for(int i=0; i<num_loose_edges; i++)
            {
                // assert(num_faces<EPA_MAX_NUM_FACES);
                if(num_faces>=EPA_MAX_NUM_FACES) break;
                faces[num_faces][0] = loose_edges[i][0];
                faces[num_faces][1] = loose_edges[i][1];
                faces[num_faces][2] = p;
                faces[num_faces][3] = ((loose_edges[i][0]-loose_edges[i][1]).cross(loose_edges[i][0]-p)).normalized();

                //Check for wrong normal to maintain CCW winding
                float bias = 0.000001; //in case dot result is only slightly < 0 (because origin is on face)
                if(faces[num_faces][0].dot(faces[num_faces][3]) + bias < 0){
                    vec3 temp = faces[num_faces][0];
                    faces[num_faces][0] = faces[num_faces][1];
                    faces[num_faces][1] = temp;
                    faces[num_faces][3] = -faces[num_faces][3];
                }
                num_faces++;
            }
        } //End for iterations
        //printf("EPA did not converge\n");
        //Return most recent closest point
        return false;
    }




};

#endif // GJK_HEADER