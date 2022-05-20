#ifndef BVH_H
#define BVH_H

#include <vector>

#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

using vec3 = glm::vec3;

namespace bvh
{
    template <typename PayLoad>
    struct Ray
    {
        Ray() = default;
        Ray(vec3 &origin, vec3 &direction) : origin(origin), direction(direction) {}
        Ray(vec3 &&origin, vec3 &&direction) : origin(origin), direction(direction) {}
        vec3 origin;
        vec3 direction;
        float t = FLT_MAX;
        PayLoad data;
    };

    struct BoundingBox
    {
        void grow(vec3 &vec)
        {
            min = glm::min(min, vec);
            max = glm::max(max, vec);
        }
        void grow(BoundingBox &box)
        {
            grow(box.min);
            grow(box.max);
        }
        float area()
        {
            vec3 extent = max - min;
            return extent.x * extent.y + extent.y * extent.z + extent.z * extent.x;
        }
        template <typename PayLoad>
        bool intersect(const Ray<PayLoad> &ray)
        {
            float tx1 = (min.x - ray.origin.x) / ray.direction.x, tx2 = (max.x - ray.origin.x) / ray.direction.x;
            float tmin = glm::min(tx1, tx2), tmax = glm::max(tx1, tx2);
            float ty1 = (min.y - ray.origin.y) / ray.direction.y, ty2 = (max.y - ray.origin.y) / ray.direction.y;
            tmin = glm::max(tmin, glm::min(ty1, ty2)), tmax = glm::min(tmax, glm::max(ty1, ty2));
            float tz1 = (min.z - ray.origin.z) / ray.direction.z, tz2 = (max.z - ray.origin.z) / ray.direction.z;
            tmin = glm::max(tmin, glm::min(tz1, tz2)), tmax = glm::min(tmax, glm::max(tz1, tz2));
            return tmax >= tmin && tmin < ray.t && tmax > 0;
        }
        bool intersect(const BoundingBox &box)
        {
            return (min.x <= box.max.x && max.x >= box.min.x) &&
                   (min.y <= box.max.y && max.y >= box.min.y) &&
                   (min.z <= box.max.z && max.z >= box.min.z);
        }
        vec3 min{FLT_MAX}, max{-FLT_MIN};
    };
    
    struct BvhNode
    {
        BoundingBox box;
        uint leftFirst, primitiveCount;
        bool isLeaf() { return primitiveCount > 0; }  
    };
    
    template <typename Primitive>
    class BVH
    {
    public:
        BVH() = default;

        void BVH_builder(std::vector<Primitive> &primitives)
        {
            BVH::primitives = primitives;
            buildBVH();
        }
        template <typename PayLoad>
        bool intersect(Ray<PayLoad> &ray)
        {
            return intersect(ray, rootNodeID);
        }
        bool intersect(Primitive &primitive)
        {
            BoundingBox box;
            Primitive::boundingBox(primitive, box.min, box.max);
            return intersect(box, primitive, rootNodeID);
        }
    private:
        template <typename PayLoad>
        bool intersect(Ray<PayLoad> &ray, const uint nodeID)
        {
            bool anyIntersection = false;
            BvhNode &node = bvhNodes[nodeID];
            if (!node.box.intersect(ray)) return anyIntersection;
            if (node.isLeaf())
            {
                for (uint i = 0; i < node.primitiveCount; i++)
                {
                    if (Primitive::intersect(ray, primitives[primitiveIDs[node.leftFirst + i]]))
                        anyIntersection = true;   
                }
            }
            else
            {
                if (intersect(ray, node.leftFirst))
                    anyIntersection = true;
                if (intersect(ray, node.leftFirst + 1))
                    anyIntersection = true;
            }
            return anyIntersection;
        }

        bool intersect(const BoundingBox &box, const Primitive &primitive, uint nodeID)
        {
            bool anyIntersection = false;
            BvhNode &node = bvhNodes[nodeID];
            if (!node.box.intersect(box)) return anyIntersection;
            if (node.isLeaf())
            {
                for (uint i = 0; i < node.primitiveCount; i++)
                {
                    if (Primitive::intersect(primitives[primitiveIDs[node.leftFirst + i]], primitive))
                        anyIntersection = true;
                    
                }
            }
            else
            {
                if (intersect(box, primitive, node.leftFirst))
                    anyIntersection = true;
                if (intersect(box, primitive, node.leftFirst + 1))
                    anyIntersection = true;
            }
            return anyIntersection;
        }

        void updateNodeBound(uint nodeID)
        {
            BvhNode &node = bvhNodes[nodeID];
            for (uint first = node.leftFirst, i = 0; i < node.primitiveCount; i++)
            {
                uint leafPrimitiveID = primitiveIDs[first + i];
                Primitive &leafPrimitive = primitives[leafPrimitiveID];
                Primitive::boundingBox(leafPrimitive, node.box.min, node.box.max);
            }
        }

        void subdivideBVH(uint nodeID)
        {
            BvhNode &node = bvhNodes[nodeID];
            if (node.primitiveCount <= 2) return;
            vec3 extent = node.box.max - node.box.min;
            int axis = 0;
            if (extent.y > extent.x) axis = 1;
            if (extent.z > extent[axis]) axis = 2;
            float splitPos = node.box.min[axis] + extent[axis] * 0.5;

            uint i = node.leftFirst;               // first primitive
            uint j = i + node.primitiveCount - 1;  // last primitive

            while (i <= j)
            {
                if (Primitive::centroid(primitives[primitiveIDs[i]])[axis] < splitPos)
                    i++;
                else
                    std::swap(primitiveIDs[i], primitiveIDs[j--]);    
            }
            uint leftCount = i - node.leftFirst;
            if (leftCount == 0 || leftCount == node.primitiveCount) return;

            uint leftChildID = nodesUsed++;
            uint rightChildID = nodesUsed++;

            bvhNodes[leftChildID].leftFirst = node.leftFirst;
            bvhNodes[leftChildID].primitiveCount = leftCount;
            bvhNodes[rightChildID].leftFirst = i;
            bvhNodes[rightChildID].primitiveCount = node.primitiveCount - leftCount;
            node.leftFirst = leftChildID;
            node.primitiveCount = 0;
            
            updateNodeBound(leftChildID);
            updateNodeBound(rightChildID);
            
            subdivideBVH(leftChildID);
            subdivideBVH(rightChildID);
        }
        
        void buildBVH()
        {
            primitiveIDs.resize(primitives.size());
            for (int i = 0; i < primitives.size(); i++) primitiveIDs[i] = i;
            bvhNodes.resize(primitives.size() * 2);
            BvhNode &rootNode = bvhNodes[rootNodeID];
            rootNode.leftFirst = 0;
            rootNode.primitiveCount = primitives.size();
            updateNodeBound(rootNodeID);
            subdivideBVH(rootNodeID);
        }

    private:
        std::vector<Primitive> primitives;
        std::vector<uint> primitiveIDs;
        std::vector<BvhNode> bvhNodes;
        uint rootNodeID = 0;
        uint nodesUsed = 1;
    };

} // namespace bvh


#endif