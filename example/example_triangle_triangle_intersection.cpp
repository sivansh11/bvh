#include "bvh.h"
#include "triangle_triangle_intersection.h"  // this is not my code, I cant find the author, please dont use this in your project incase you want to distribute it

#include <iostream>

#include <glm/glm.hpp>

struct HitRecord
{
    glm::vec3 source{}, target{};
    int coplanar{};
};

struct Triangle
{
    Triangle() = default;
    Triangle(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2) : vert0(v0), vert1(v1), vert2(v2) { center = (v0 + v1 + v2) / 3.0f; }
    glm::vec3 vert0;
    glm::vec3 vert1;
    glm::vec3 vert2;
    glm::vec3 center;
    
    // these are some custom functions that ever bvh must contain, Note all of these are static and the first parameter is always a refrence of the Primitive type

    // this function takes in a min and max vec3 refrence.
    // the perpouse of the function is to return the bounding box of the current primitive 
    static void boundingBox(Triangle &tri, glm::vec3 &min, glm::vec3 &max)
    {
        min = glm::min(min, tri.vert0);
        min = glm::min(min, tri.vert1);
        min = glm::min(min, tri.vert2);
        max = glm::max(max, tri.vert0);
        max = glm::max(max, tri.vert1);
        max = glm::max(max, tri.vert2);
    }

    // this function just returns the middle of the primitive
    static glm::vec3 centroid(Triangle &tri)
    {
        return tri.center;
    }

    // returns if primitive-primitive intersection took place
    static bool intersect(Triangle &tri, HitRecord &hitRecord, Triangle &other)
    {
        int didIntersect = tri_tri_intersection_test_3d((float*)(&tri.vert0), (float*)(&tri.vert1), (float*)(&tri.vert2),
                                     (float*)(&other.vert0), (float*)(&other.vert1), (float*)(&other.vert2),
                                     &hitRecord.coplanar, (float*)(&hitRecord.source), (float*)(&hitRecord.target));
        return didIntersect;
    }
};

// helper function to use std::cout with glm::vec3
std::ostream& operator<<(std::ostream& out, glm::vec3& vec)
{
    return out << vec.x << ' ' << vec.y << ' ' << vec.z;
}

int main()
{
    std::vector<Triangle> tris {
        Triangle{{-.5, -.5, 1}, {.5, -.5, 1}, {0, .5, 1}},
        // ...   there can be any amount of triangles present in this
    };
    bvh::BVH<Triangle> bvh;
    bvh.BVH_builder(tris);  // bvh only stores a pointer to the vector, so the vector must not be deallocated 

    Triangle tri1{{0, -.5, 0}, {0, -.5, 1}, {0, .5, .5}};

    HitRecord hitRecord1;
    if (bvh.intersect(tri1, hitRecord1))
    {
        std::cout << "yes, more info\nIs coplanar:" << hitRecord1.coplanar << "\nTarget:" << hitRecord1.target << "\nSource:" << hitRecord1.source << '\n';
    }
    std::cout << "\n";

    Triangle tri2{{-1, -1, 1}, {1, -1, 1}, {0, 1, 1}};

    HitRecord hitRecord2;
    if (bvh.intersect(tri2, hitRecord2))
    {
        std::cout << "yes, more info\nIs coplanar:" << hitRecord2.coplanar << "\nTarget:" << hitRecord2.target << "\nSource:" << hitRecord2.source << '\n';
    }


    return 0;
}