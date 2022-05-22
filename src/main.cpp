#define STATS
#include "bvh.h"
#include "triangle_triangle_intersection.h"
#include "timer.h"
#include "general_utils.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tinyobjloader.h"

#include <iostream>
#include <thread>

// extra data that is attached to a ray
struct PayLoad
{
    // vec3 col;
};

struct Triangle
{
    Triangle() = default;
    Triangle(vec3 v0, vec3 v1, vec3 v2) : vert0(v0), vert1(v1), vert2(v2) {}
    vec3 vert0;
    vec3 vert1;
    vec3 vert2;
    vec3 center;

    // every primitive must have 3 functions defined (4th is optional for primitive-primitive intersection, example triangle-triangle intersection)

    // intersect function intersects a ray to the primitve, this is where you can add extra information to the ray
    static void intersect(const Triangle &tri, bvh::Ray<PayLoad> &ray)
    {
        const vec3 edge1 = tri.vert1 - tri.vert0;
        const vec3 edge2 = tri.vert2 - tri.vert0;
        const vec3 h = glm::cross( ray.direction, edge2 );
        const float a = dot( edge1, h );
        if (a > -0.0001f && a < 0.0001f) return; // ray parallel to triangle
        const float f = 1 / a;
        const vec3 s = ray.origin - tri.vert0;
        const float u = f * dot( s, h );
        if (u < 0 || u > 1) return;
        const vec3 q = cross( s, edge1 );
        const float v = f * dot( ray.direction, q );
        if (v < 0 || u + v > 1) return;
        const float t = f * dot( edge2, q );
        if (t < 0.0001f) return;
        if (ray.t < t) return;
        ray.t = t;
        // payload data 
        // ray.data.col = tri.col;
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

    // this intersect function is used to detect primitive-primitive intersect (in this case Triangle-Triangle)
    static bool intersect(const Triangle &tri1, const Triangle &tri2)
    {
        float t[3];
        float s[3];
        int isCoplaner;
        return tri_tri_intersection_test_3d((float*)(&tri1.vert0), (float*)(&tri1.vert1), (float*)(&tri1.vert2), (float*)(&tri2.vert0), (float*)(&tri2.vert1), (float*)(&tri2.vert2), &isCoplaner, s, t);
    }
};  

class Camera
{
public:
    Camera() = default;
    Camera(vec3 &&from, vec3 &&at, vec3 &&up, float aspectRatio, float fov) : aspectRatio(aspectRatio), fov(fov)
    {
        set(from, at, up, aspectRatio, fov);
    }
    Camera(vec3 &from, vec3 &at, vec3 &&up, float aspectRatio, float fov) : aspectRatio(aspectRatio), fov(fov)
    {
        set(from, at, up, aspectRatio, fov);
    }
    void set(vec3 from, vec3 at, vec3 up, float aspectRatio, float fov)
    {
        float theta = glm::radians(fov);
        float h = glm::tan(theta / 2);
        vpHeight = 2.0f * h;
        vpWidth = aspectRatio * vpHeight;

        vec3 w = glm::normalize(from - at);
        vec3 u = glm::normalize(glm::cross(up, w));
        vec3 v = glm::cross(w, u);

        origin = from;
        horizontal = vpWidth * u;
        vertical = vpHeight * v;
        lowerLeftCorner = origin - (horizontal / 2.f) - (vertical / 2.f) - w;
    }
    bvh::Ray<PayLoad> getRay(float u, float v)
    {
        return bvh::Ray<PayLoad>(origin, lowerLeftCorner + (u * horizontal) + (v * vertical) - origin);
    }

private:
    vec3 origin;
    vec3 lowerLeftCorner;
    vec3 horizontal;
    vec3 vertical;
    float aspectRatio;
    float fov;
    float vpHeight;
    float vpWidth;
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

float randFloat()
{
    return float(rand()) / RAND_MAX;
}

void load_obj_file(const char *path, std::vector<Triangle> &tris)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, path))
    {
        throw std::runtime_error(warn + err);
    }

    std::vector<vec3> verts;
    std::vector<uint> indices;

    for (auto& shape: shapes)
    {
        for (auto& index: shape.mesh.indices)
        {
            vec3 vert{};

            vert = {attrib.vertices[3 * index.vertex_index + 0],
                    attrib.vertices[3 * index.vertex_index + 1],
                    attrib.vertices[3 * index.vertex_index + 2]};

            verts.push_back(vert);
            indices.push_back(indices.size());
        }
    }

    for (int i=0; i < indices.size(); i+=3)
    {
        tris.emplace_back(verts[i], verts[i + 1], verts[i + 2]);
    }
}

std::ostream& operator<<(std::ostream& out, vec3& vec)
{
    return out << vec.x << ' ' << vec.y << ' ' << vec.z << '\n';
}

int main()
{
    const int width = 640;
    const int height = 640;
    const int N = 10000;
    const int numThreads = 1;

    TimeIt timer;

    timer.from();
    std::vector<Triangle> tris;
    // #ifndef NDEBUG
    // load_obj_file("../assets/shotgun.obj", tris);
    // #else
    // load_obj_file("assets/shotgun.obj", tris);
    // #endif

    // for (auto& tri: tris)
    // {
    //     std::cout << tri.vert0 << ' ' << tri.vert1 << ' ' << tri.vert2 << '\n';
    // }
    tris.resize(N);
    for (int i = 0; i < N; i++)
    {
        vec3 r0{randFloat(), randFloat(), randFloat()};
        vec3 r1{randFloat(), randFloat(), randFloat()};
        vec3 r2{randFloat(), randFloat(), randFloat()};
        tris[i].vert0 = r0 * 49.0f - vec3{25};
        tris[i].vert1 = tris[i].vert0 + r1;
        tris[i].vert2 = tris[i].vert0 + r2;
        tris[i].center = (tris[i].vert0 + tris[i].vert1 + tris[i].vert2) * 0.3333333f;
        // std::cout << tris[i].vert0 << ' ' << tris[i].vert1 << ' ' << tris[i].vert2 << '\n';
    }
    std::cerr << "Triangle creation took: " << (timer.now() / 1000) << '\n'; 

    bvh::BVH<Triangle> bvh;

    timer.from();
    // bvh.BVH_builder(tris); std::cerr << "---BVH---\n";
    bvh.BVH_SAH_builder(tris, 8); std::cerr << "---BVH + SAH---\n";
    // bvh.BVH_BIN_builder(tris, 8); std::cerr << "---BVH + BIN---\n";
    std::cerr << "BVH build took: " << (timer.now() / 1000) << '\n'; 

    uint32_t data[width * height];

    Camera camera;
    camera.set(vec3{0, 0, -50}, vec3{0, 0, 0}, vec3{0, 1, 0}, float(width) / height, 90.0);

    timer.from();

    std::vector<std::thread> threads;

    float avjIntersections = 0;

    for (int n = 0; n < numThreads; n++)
    {
        auto task = [&data, &numThreads, &camera, &bvh, &width, &height, &avjIntersections, n]()
        {
            int columnsPerThread = width / numThreads;
            for (int i= n * columnsPerThread; i <= (n + 1) * columnsPerThread && i<width; i++) for (int j=0; j<height; j++)
            {
                float U = float(i) / (width - 1);
                float V = float(j) / (height - 1);

                bvh::Ray<PayLoad> r = camera.getRay(U, V);
                bvh.intersect(r);
                uint c = 500 - (int)(r.t * 42);
                if (r.t != FLT_MAX)
                {
                    data[(j) * width + (i)] = color(c * 0x10101, c * 0x10101, c * 0x10101, 255);
                    // data[(j) * width + (i)] = color(255, 255, 255, 255);
                }
                else
                {
                    data[(j) * width + (i)] = 0;
                }
                avjIntersections += r.intersections;
            }
        };
        threads.emplace_back(task);
    }
    for (int i=0; i<threads.size(); i++)
    {
        threads[i].join();
    }
    std::cerr << "Image render took: " << (timer.now() / 1000) << '\n'; 

    std::cerr << "Average Intersections: " << avjIntersections / (width * height) << '\n';

    std::string outImage;

    outImage += "P3\n" + std::to_string(width) + ' ' + std::to_string(height) + "\n255\n";
    for (int j = 0; j < height; ++j) {
        for (int i = 0; i < width; ++i) {
            uint8_t col[4];
            colorConversion(data[j * width + i], col);
            outImage += std::to_string(int(col[0])) + ' ' + std::to_string(int(col[1])) + ' ' + std::to_string(int(col[2])) + '\n';
        }
    }
    utils::save_string(outImage, "../image.ppm");
    return 0;
}