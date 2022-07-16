#include "bvh.h"

#include <iostream>

#include <glm/glm.hpp>

struct HitRecord
{

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

    // this function takes in a refrence of a Triangle(Primitive), a refrence of a ray and a refrence of a HitRecord
    // note that it does not return anythin
    // please see the update section
    template <typename HitRecord>
    static void intersect(Triangle &tri, bvh::Ray &ray, HitRecord &hitRecord)
    {
        const glm::vec3 edge1 = tri.vert1 - tri.vert0;
        const glm::vec3 edge2 = tri.vert2 - tri.vert0;
        const glm::vec3 h = glm::cross(ray.direction, edge2);
        const float a = glm::dot(edge1, h);
        if (glm::abs(a) < 0.0001f) return;
        const float f = 1.0f / a;
        const glm::vec3 s = ray.origin - tri.vert0;
        const float u = f * glm::dot(s, h);
        if (u < 0 || u > 1) return;
        const glm::vec3 q = glm::cross(s, edge1);
        const float v = f * glm::dot(ray.direction, q);
        if (v < 0 || u + v > 1) return;
        const float t = f * glm::dot(edge2, q);
        if (t < 0.0001f) return;

        if (ray.t < t) return;  // if the ray contains a better/closer intersection, then leave and dont update anything
        
        // update section
        // ray update
        ray.t = t;  // only need to update t 

        // hitRecord update
        // hitRecord.p = ray.at(ray.t);  // Note: We cant store the hitPoint with bvhInstances becasue the 
                                         // ray is modified and it isnt the original ray, the origin and direction is different
    }

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

    bvh::BVHInstance<Triangle> instance{&bvh}; // expicitly takes in a pointer to a bvh
                                               // there can be multiple instances that point to the same bvh

    glm::mat4 transform{1.0f};
    transform = glm::translate(transform, {0, 0, 2});
    transform = glm::scale(transform, {2, 2, 1});  // the last component is 1 as scaling a 2d object in a 3d world doesnt makes sense, also it messes up some calculations

    instance.setTransform(transform); // sets the transform for the particular instance

    bvh::Ray ray{{0, 0, 0}, {0, 0, 1}};  // first origin then direction

    HitRecord hitRecord;
    if (instance.intersect(ray, hitRecord))   // instead of intersecting with the bvh, we use the instance
    {
        glm::vec3 p = ray.at(ray.t);
        std::cout << "Yes at:" << p << '\n';  
    }


    return 0;
}