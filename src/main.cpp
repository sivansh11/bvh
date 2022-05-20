#include "bvh.h"
#include "triangle_triangle_intersection.h"

#include <iostream>

// extra data that is attached to a ray
struct PayLoad
{
    vec3 col;
};


struct Triangle
{
    Triangle() = default;
    Triangle(vec3 v0, vec3 v1, vec3 v2, vec3 col) : vert0(v0), vert1(v1), vert2(v2), col(col) {}
    vec3 vert0;
    vec3 vert1;
    vec3 vert2;
    vec3 col;
    vec3 center;

    // every primitive must have 3 functions defined (4th is optional for primitive-primitive intersection, example triangle-triangle intersection)

    // intersect function intersects a ray to the primitve, this is where you can add extra information to the ray
    static bool intersect(bvh::Ray<PayLoad> &ray, const Triangle &tri)
    {
        const vec3 edge1 = tri.vert1 - tri.vert0;
        const vec3 edge2 = tri.vert2 - tri.vert0;
        const vec3 h = glm::cross( ray.direction, edge2 );
        const float a = dot( edge1, h );
        if (a > -0.0001f && a < 0.0001f) return false; // ray parallel to triangle
        const float f = 1 / a;
        const vec3 s = ray.origin - tri.vert0;
        const float u = f * dot( s, h );
        if (u < 0 || u > 1) return false;
        const vec3 q = cross( s, edge1 );
        const float v = f * dot( ray.direction, q );
        if (v < 0 || u + v > 1) return false;
        const float t = f * dot( edge2, q );
        if (t < 0.0001f) return false;
        if (ray.t < t) return false;
        ray.t = t;
        // payload data 
        ray.data.col = tri.col;
        return true;
    }

    // boundingBox function is used to set a bounding box for a primitive, in this case, the box encloses the triangle
    static void boundingBox(Triangle &tri, vec3 &min, vec3 &max)
    {
        min = glm::min(min, tri.vert0);
        min = glm::min(min, tri.vert1);
        min = glm::min(min, tri.vert2);
        max = glm::max(max, tri.vert0);
        max = glm::max(max, tri.vert1);
        max = glm::max(max, tri.vert2);
    }

    // centroid function is used to return the center of the primitive 
    static vec3 centroid(Triangle &tri)
    {
        return tri.center;
    }

    static bool intersect(const Triangle &tri1, const Triangle &tri2)
    {
        float t[3];
        float s[3];
        int isCoplaner;
        return tri_tri_intersection_test_3d((float*)(&tri1.vert0), (float*)(&tri1.vert1), (float*)(&tri1.vert2), (float*)(&tri2.vert0), (float*)(&tri2.vert1), (float*)(&tri2.vert2), &isCoplaner, s, t);
    }
};  


void colorConversion(uint32_t col, uint8_t *newCol)
{
    *(uint32_t *)newCol = col;
}

uint32_t color(uint8_t p_r, uint8_t p_g, uint8_t p_b, uint8_t p_a)
{
    uint32_t res;
    res = p_r | p_g << 8 | p_b << 16 | p_a << 24;
    return res;
}

int main()
{
    const int width = 1200;
    const int height = 800;

    std::vector<Triangle> tris;
    tris.push_back(Triangle{vec3{-1, -1, 1}, vec3{1, -1, 1}, vec3{0, 1, 1}, vec3{0,0,255}});
    tris.push_back(Triangle{vec3{-.5, -.5, 0}, vec3{.5, -.5, 0}, vec3{0, .5, 0}, vec3{255,0,0}});
    
    bvh::BVH<Triangle> bvh;

    bvh.BVH_builder(tris);

    uint32_t data[width * height];

    vec3 camPos{0, 0, -18};
    vec3 p0{-1, 1, -2}, p1{1, 1, -2}, p2{-1, -1, -2};

    for (int i=0; i<width; i++) for (int j=0; j<height; j++)
    {
        vec3 pixelPos = p0 + (p1 - p0) * (float(i) / width) + (p2 - p0) * (float(j) / height);
        bvh::Ray<PayLoad> r;
        r.origin = camPos;
        r.direction = glm::normalize(pixelPos - camPos);
        r.data = PayLoad{};

        if (bvh.intersect(r))
        {
            vec3 col = r.data.col;
            data[j * width + i] = color(col.r, col.g, col.b, 255);
        }
        else
        {
            data[j * width + i] = 0;
        }
    }

    std::cout << "P3\n" << width << ' ' << height << "\n255\n";
    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            uint8_t col[4];
            colorConversion(data[j * width + i], col);
            std::cout << int(col[0]) << ' ' << int(col[1]) << ' ' << int(col[2]) << '\n';
        }
    }

    return 0;
}