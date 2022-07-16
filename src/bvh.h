#ifndef BVH_H
#define BVH_H

#include <vector>

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

namespace bvh
{

struct Ray
{
    glm::vec3 origin, direction, inverseDirection;
    float t = FLT_MAX;
    // anytime a ray is default constructed, the inverse direction must be manually set
    Ray() = default;
    Ray(glm::vec3 &origin, glm::vec3 &direction) : origin(origin), direction(direction), inverseDirection(1.0f / direction.x, 1.0f / direction.y, 1.0f / direction.z) {}
    Ray(glm::vec3 &&origin, glm::vec3 &&direction) : origin(origin), direction(direction), inverseDirection(1.0f / direction.x, 1.0f / direction.y, 1.0f / direction.z) {}
    glm::vec3 at(float t)
    {
        return origin + (direction * t);
    }
};

struct BoundingBox
{
    glm::vec3 min, max;
    // returns a float representing distance
    float intersect(const Ray &ray)
    {
        float tmin, tmax;
        float tx1 = (min.x - ray.origin.x) * ray.inverseDirection.x, tx2 = (max.x - ray.origin.x) * ray.inverseDirection.x;
        tmin = glm::min(tx1, tx2), tmax = glm::max(tx1, tx2);
        float ty1 = (min.y - ray.origin.y) * ray.inverseDirection.y, ty2 = (max.y - ray.origin.y) * ray.inverseDirection.y;
        tmin = glm::max(tmin, glm::min(ty1, ty2)), tmax = glm::min(tmax, glm::max(ty1, ty2));
        float tz1 = (min.z - ray.origin.z) * ray.inverseDirection.z, tz2 = (max.z - ray.origin.z) * ray.inverseDirection.z;
        tmin = glm::max(tmin, glm::min(tz1, tz2)), tmax = glm::min(tmax, glm::max(tz1, tz2));
        if (tmax >= tmin && tmin < ray.t && tmax > 0) return tmin;
        else return FLT_MAX;
    }
    // returns a bool
    bool intersect(const BoundingBox &box)
    {
        return (min.x <= box.max.x && max.x >= box.min.x) &&
               (min.y <= box.max.y && max.y >= box.min.y) &&
               (min.z <= box.max.z && max.z >= box.min.z);
    }
    void grow(glm::vec3 &vec)
    {
        min = glm::min(min, vec);
        max = glm::max(max, vec);
    }
    void grow(BoundingBox &box)
    {
        grow(box.min);
        grow(box.max);
    }
    // half area not full
    float area()
    {
        glm::vec3 e = max - min;
        return e.x * e.y + e.y * e.z + e.z * e.x;
    }
};

struct BvhNode
{
    BoundingBox box;
    uint32_t leftFirst, primitiveCount;
    bool isLeaf() { return primitiveCount > 0; }
};


// A Primitive like the name suggest is the type of geometry for which the bvh is built for, example triangles, spheres
template <typename Primitive>
class BVH
{
public:
    BVH() = default;
    // this is the naive bvh implementation, better quality bvh construction implementation will be added 
    // for eg:
    // void BVH_SAH_builder(std::vector<Primitive> &primitives)
    void BVH_builder(std::vector<Primitive> &primitives)
    {
        mPrimitives = &primitives;
        buildBVH();
    }
    
    // HitRecord is any data that you may wanna maintain about the primitive post intersection, this is defined by the client
    template <typename HitRecord>
    bool intersect(Ray &ray, HitRecord &hitRecord)
    {
        intersect(ray, mRootNodeID, hitRecord);
        return ray.t != FLT_MAX;
    }
    
    // Experimental Primitive-Primitive intersection, may not work completely / may be buggy
    // Do note that this wont be compiled if its not needed
    // TODO: make this faster
    template <typename Intersector, typename HitRecord>
    bool intersect(Intersector &intersector, HitRecord &hitRecord)
    {
        BoundingBox box;
        Intersector::boundingBox(intersector, box.min, box.max);
        return intersect(intersector, box, mRootNodeID, hitRecord);
    }

    // if the data of the primitive changes slightly, you could just refit it
    // just remember that only the data values may change like the positions of vertices, there must not be additions or subtractions to the number of primitives, 
    // for that you must rebuild a new bvh from scratch
    // Note: The quality of the rebuilt bvh is worse than what it was before
    void refitBVH()
    {
        for (int i = mNodesUsed - 1; i >= 0; i--) if (i != 1)
        {
            BvhNode &node = mBvhNodes[i];
            if (node.isLeaf())
            {
                updateNodeBound(i);
                continue;
            }
            BvhNode &leftChild = mBvhNodes[node.leftFirst];
            BvhNode &rightChild = mBvhNodes[node.leftFirst + 1];
            node.box.min = glm::min(leftChild.box.min, rightChild.box.min);
            node.box.max = glm::max(leftChild.box.max, rightChild.box.max);
        }
    }
    BvhNode* getBvhNodes() { return mBvhNodes.data(); }

private:
    template <typename HitRecord>
    void intersect(Ray &ray, uint32_t nodeID, HitRecord &hitRecord)
    {
        BvhNode *node = &mBvhNodes[nodeID], *stack[64];
        int stackPtr = 0;
        while (1)
        {
            if (node->isLeaf())
            {
                for (int i = 0; i < node->primitiveCount; i++)
                {
                    Primitive::intersect(mPrimitives->at(mPrimitiveIDs[node->leftFirst + i]), ray, hitRecord);
                }
                if (stackPtr == 0)
                {
                    break;
                }
                else
                {
                    node = stack[--stackPtr];
                }
                continue;
            }
            BvhNode *child1 = &mBvhNodes[node->leftFirst];
            BvhNode *child2 = &mBvhNodes[node->leftFirst + 1];
            float dist1 = child1->box.intersect(ray);
            float dist2 = child2->box.intersect(ray);
            if (dist1 > dist2)
            {
                std::swap(dist1, dist2);
                std::swap(child1, child2);
            }
            if (dist1 == FLT_MAX)
            {
                if (stackPtr == 0)
                {
                    break;
                }
                else
                {
                    node = stack[--stackPtr];
                }
            }
            else
            {
                node = child1;
                if (dist2 != FLT_MAX)
                {
                    assert(stackPtr < 62); // need to increase stack size from 64 to something bigger
                    stack[stackPtr++] = child2;
                }
            } 
        }
    }

    template <typename Intersector, typename HitRecord> 
    bool intersect(Intersector &intersector, BoundingBox &box, uint32_t nodeID, HitRecord &hitRecord)
    {
        bool anyIntersection = false;
        BvhNode &node = mBvhNodes[nodeID];
        if (!node.box.intersect(box)) return anyIntersection;
        if (node.isLeaf())
        {
            for (uint i = 0; i < node.primitiveCount; i++)
            {
                if (Primitive::intersect(mPrimitives->at(mPrimitiveIDs[node.leftFirst + i]), hitRecord, intersector))
                    anyIntersection = true;
            }
        }
        else
        {
            if (intersect(intersector, box, node.leftFirst, hitRecord))
                anyIntersection = true;
            if (intersect(intersector, box, node.leftFirst + 1, hitRecord))
                anyIntersection = true;
        }
        return anyIntersection;
    }

    void updateNodeBound(uint32_t nodeID)
    {
        BvhNode &node = mBvhNodes[nodeID];
        node.box.min = glm::vec3{FLT_MAX};
        node.box.max = glm::vec3{-FLT_MIN};
        for (int first = node.leftFirst, i = 0; i < node.primitiveCount; i++)
        {
            uint32_t leafPrimitiveID = mPrimitiveIDs[first + i];
            Primitive &leafPrimitive = mPrimitives->at(leafPrimitiveID);
            Primitive::boundingBox(leafPrimitive, node.box.min, node.box.max);
        }
    }

    void subdivideBVH(uint32_t nodeID)
    {
        BvhNode &node = mBvhNodes[nodeID];
        if (node.primitiveCount <= 2) return;
        glm::vec3 e = node.box.max - node.box.min;
        int axis = 0;
        if (e.y > e.x) axis = 1;
        if (e.z > e[axis]) axis = 2;
        float splitPos = node.box.min[axis] + e[axis] * 0.5f;

        int i = node.leftFirst;              // first primitive
        int j = i + node.primitiveCount - 1; // last primitive

        while (i <= j)
        {
            if (Primitive::centroid(mPrimitives->at(mPrimitiveIDs[i]))[axis] < splitPos)
                i++;
            else
                std::swap(mPrimitiveIDs[i], mPrimitiveIDs[j--]);
        }
        uint32_t leftCount = i - node.leftFirst;
        if (leftCount == 0 || leftCount == node.primitiveCount) return;

        uint32_t leftChildID = mNodesUsed++;
        uint32_t rightChildID = mNodesUsed++;

        mBvhNodes[leftChildID].leftFirst = node.leftFirst;
        mBvhNodes[leftChildID].primitiveCount = leftCount;
        mBvhNodes[rightChildID].leftFirst = i;
        mBvhNodes[rightChildID].primitiveCount = node.primitiveCount - leftCount;
        node.leftFirst = leftChildID;
        node.primitiveCount = 0;

        updateNodeBound(leftChildID);
        updateNodeBound(rightChildID);

        subdivideBVH(leftChildID);
        subdivideBVH(rightChildID);
    }

    void buildBVH()
    {
        mPrimitiveIDs.resize(mPrimitives->size());
        for (int i = 0; i < mPrimitives->size(); i++) mPrimitiveIDs[i] = i;
        mBvhNodes.resize(mPrimitives->size() * 2);
        BvhNode &rootNode = mBvhNodes[mRootNodeID];
        rootNode.leftFirst = 0;
        rootNode.primitiveCount = mPrimitives->size();
        updateNodeBound(mRootNodeID);
        subdivideBVH(mRootNodeID);
    }

private:
    std::vector<Primitive> *mPrimitives;
    std::vector<uint32_t> mPrimitiveIDs;
    std::vector<BvhNode> mBvhNodes;
    uint32_t mRootNodeID = 0;
    uint32_t mNodesUsed = 1;
};

// BvhInstance is used for transforming a prebuilt bvh, there can be multiple bvhinstances for 1 bvh 
// I would use a bvh per model and maintain multiple bvhinstances for each instance (even if there is 1 instance, keeping it as a bvhinstance is better as you can apply transformations)
// note that BvhInstance does not have the experimental primitive-primitive intersection function
template <typename Primitive>
class BVHInstance
{
public:
    BVHInstance() = default;
    BVHInstance(BVH<Primitive> *blas) : mBlas(blas) 
    { 
        glm::mat4 transform{1.0f};
        setTransform(transform); 
    }

    // set trasnform takes in a transformation matrix that has to be applied to the model in question
    void setTransform(glm::mat4 &transform)
    {
        mInvTransform = glm::inverse(transform);
        glm::vec3 min = mBlas->getBvhNodes()[0].box.min, max = mBlas->getBvhNodes()[0].box.max;
        for (int i = 0; i < 8; i++)
        {
            glm::vec3 pos = {i & 1 ? max.x : min.x, 
                             i & 2 ? max.y : min.y,
                             i & 4 ? max.z : min.z};
            pos = transform * glm::vec4(pos, 1.0f);
            mBounds.grow(pos);
        }
    }
    
    template <typename HitRecord>
    bool intersect(Ray &ray, HitRecord &hitRecord)
    {
        Ray backupRay = ray;
        ray.origin = mInvTransform * glm::vec4(ray.origin, 1.0f);
        ray.direction = mInvTransform * glm::vec4(ray.direction, 0.0f);
        ray.inverseDirection = {1.0f / ray.direction.x, 1.0f / ray.direction.y, 1.0f / ray.direction.z};

        bool didIntersect = mBlas->intersect(ray, hitRecord);

        backupRay.t = ray.t;
        ray = backupRay;
        return didIntersect;
    }   

    // requirement of this function will be explained in the example
    template <typename HitRecord>
    static void intersect(BVHInstance &bvhInstance, Ray &ray, HitRecord &hitRecord)
    {
        Ray backupRay = ray;
        ray.origin = bvhInstance.mInvTransform * glm::vec4(ray.origin, 1.0f);
        ray.direction = bvhInstance.mInvTransform * glm::vec4(ray.direction, 0.0f);
        ray.inverseDirection = {1.0f / ray.direction.x, 1.0f / ray.direction.y, 1.0f / ray.direction.z};

        bvhInstance.mBlas->intersect(ray, hitRecord);

        backupRay.t = ray.t;
        ray = backupRay;
    }

    // requirement of this function will be explained in the example
    static void boundingBox(BVHInstance &bvhInstance, glm::vec3 &min, glm::vec3 &max)
    {
        min = bvhInstance.mBounds.min;
        max = bvhInstance.mBounds.max;
    }

    // requirement of this function will be explained in the example
    static glm::vec3 centroid(BVHInstance &bvhInstance)
    {
        return (bvhInstance.mBounds.min + bvhInstance.mBounds.max) / 2.0f;
    }

    BoundingBox& getBounds() { return mBounds; }
    glm::mat4& getInvTransform() { return mInvTransform; }

private:
    BVH<Primitive> *mBlas{};
    glm::mat4 mInvTransform;
    BoundingBox mBounds{};
};

// Typically a tlas is built bottom to top which yields much higher quality of bvh
// I havent added that yet, so I am using the default method, it may be added later
template <typename Primitive> using TLAS = BVH<BVHInstance<Primitive>>;

} // namespace bvh


#endif