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
    {   // ignore this messy code, will be removed later 
        Ray() = default;
        Ray(vec3 &origin, vec3 &direction) : origin(origin), direction(direction), invDirection(1.0f / direction.x, 1.0f / direction.y, 1.0f / direction.z) {}
        Ray(vec3 origin, vec3 direction) : origin(origin), direction(direction), invDirection(1.0f / direction.x, 1.0f / direction.y, 1.0f / direction.z) {}
        vec3 origin;
        vec3 direction;
        vec3 invDirection;
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
        float intersect(Ray<PayLoad> &ray)
        {
            float tx1 = (min.x - ray.origin.x) / ray.direction.x, tx2 = (max.x - ray.origin.x) / ray.direction.x;
            float tmin = glm::min(tx1, tx2), tmax = glm::max(tx1, tx2);
            float ty1 = (min.y - ray.origin.y) / ray.direction.y, ty2 = (max.y - ray.origin.y) / ray.direction.y;
            tmin = glm::max(tmin, glm::min(ty1, ty2)), tmax = glm::min(tmax, glm::max(ty1, ty2));
            float tz1 = (min.z - ray.origin.z) / ray.direction.z, tz2 = (max.z - ray.origin.z) / ray.direction.z;
            tmin = glm::max(tmin, glm::min(tz1, tz2)), tmax = glm::min(tmax, glm::max(tz1, tz2));
            if (tmax >= tmin && tmin < ray.t && tmax > 0) return tmin;
            else
                return FLT_MAX;
        }
        bool intersect(const BoundingBox &box)
        {
            return (min.x <= box.max.x && max.x >= box.min.x) &&
                   (min.y <= box.max.y && max.y >= box.min.y) &&
                   (min.z <= box.max.z && max.z >= box.min.z);
        }
        vec3 min{FLT_MAX}, max{-FLT_MIN};
    };
    
    struct Bin
    {
        BoundingBox box;
        int primitiveCount = 0;
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

        void BVH_SAH_builder(std::vector<Primitive> &primitive, int num_intervals = 16)
        {
            BVH::primitives = primitive;
            buildSAH(num_intervals);
        }
        void BVH_BIN_builder(std::vector<Primitive> &primitive, int num_intervals = 16)
        {
            BVH::primitives = primitive;
            buildBIN(num_intervals);
        }
        template <typename PayLoad>
        void intersect(Ray<PayLoad> &ray)
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
        void intersect(Ray<PayLoad> &ray, const uint nodeID)
        {
            BvhNode *node = &bvhNodes[nodeID], *stack[64];
            uint stackPtr = 0;
            while (true)
            {
                if (node->isLeaf())
                {
                    for (uint i = 0; i < node->primitiveCount; i++)
                    {
                        Primitive::intersect(primitives[primitiveIDs[node->leftFirst + i]], ray);
                    }
                    if (stackPtr == 0)
                        break;
                    else
                    {
                        node = stack[--stackPtr];
                    }
                    continue;
                }
                BvhNode *child1 = &bvhNodes[node->leftFirst];
                BvhNode *child2 = &bvhNodes[node->leftFirst + 1];
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
                        break;
                    else 
                        node = stack[--stackPtr];
                }
                else
                {
                    node = child1;
                    if (dist2 != FLT_MAX) 
                        stack[stackPtr++] = child2;
                }
            }
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
                {
                    assert(j < primitives.size());
                    std::swap(primitiveIDs[i], primitiveIDs[j--]); 
                }   
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

        float findBestSplitPlaneSAH(BvhNode &node, int &axis, float &splitPos, int num_intervals)
        {
            float inv = 1.0f / num_intervals;
            float bestCost = FLT_MAX;
            for (int a = 0; a < 3; a++)
            {
                float boundsMin = FLT_MAX;
                float boundsMax = -FLT_MAX;
                for (int i = 0; i < node.primitiveCount; i++)
                {
                    Primitive &primitive = primitives[primitiveIDs[node.leftFirst + i]];
                    boundsMin = glm::min(boundsMin, Primitive::centroid(primitive)[a]);
                    boundsMax = glm::max(boundsMax, Primitive::centroid(primitive)[a]);
                }
                if (boundsMin == boundsMax) continue;
                float scale = (boundsMax - boundsMin) * inv;
                for (uint i = 0; i < num_intervals; i++)
                {
                    float candidatePos = boundsMin + i * scale;
                    float cost = evaluateSAH(node, a, candidatePos);
                    if (cost < bestCost)
                    {
                        splitPos = candidatePos;
                        axis = a;
                        bestCost = cost;
                    }
                }
            }
            return bestCost;
        }

        float findBestSplitPlaneBIN(BvhNode &node, int &axis, float &splitPos, int num_intervals)
        {
            float bestCost = FLT_MAX;
            for (int a = 0; a < 3; a++)
            {
                float boundsMin = FLT_MAX;
                float boundsMax = -FLT_MAX;
                for (int i = 0; i < node.primitiveCount; i++)
                {
                    Primitive &primitive = primitives[primitiveIDs[node.leftFirst + i]];
                    boundsMin = glm::min(boundsMin, Primitive::centroid(primitive)[a]);
                    boundsMax = glm::max(boundsMax, Primitive::centroid(primitive)[a]);
                }
                if (boundsMin == boundsMax) continue;

                Bin bin[num_intervals];
                float scale = float(num_intervals) / (boundsMax - boundsMin);
                for (uint i = 0; i < node.primitiveCount; i++)
                {
                    Primitive &primitive = primitives[primitiveIDs[node.leftFirst + i]];
                    int binID = glm::min(num_intervals - 1, int((Primitive::centroid(primitive)[a] - boundsMin) * scale));
                    bin[binID].primitiveCount++;
                    BoundingBox primitiveBox;
                    Primitive::boundingBox(primitive, primitiveBox.min, primitiveBox.max);
                    bin[binID].box.grow(primitiveBox);
                }

                float leftArea[num_intervals - 1], rightArea[num_intervals - 1];
                int leftCount[num_intervals - 1], rightCount[num_intervals - 1];
                BoundingBox leftBox, rightBox;
                int leftSum = 0, rightSum = 0;
                for (int i = 0; i < num_intervals - 1; i++)
                {
                    leftSum += bin[i].primitiveCount;
                    leftCount[i] = leftSum;
                    leftBox.grow(bin[i].box);
                    leftArea[i] = leftBox.area();
                    rightSum += bin[num_intervals - 1 - i].primitiveCount;
                    rightCount[num_intervals - 2 - i] = rightSum;
                    rightBox.grow(bin[num_intervals - 1 - i].box);
                    rightArea[num_intervals - 2 - i] = rightBox.area();
                }

                scale = (boundsMax - boundsMin) / num_intervals;
                for (int i = 0; i < num_intervals - 1; i++)
                {
                    float planeCost = leftCount[i] * leftArea[i] + rightCount[i] * rightArea[i];
                    if (planeCost < bestCost)
                    {
                        axis = a;
                        splitPos = boundsMin + scale * (i + 1);
                        bestCost = planeCost;
                    }
                }
            }
            return bestCost;
        }

        float evaluateSAH(BvhNode &node, int axis, float pos)
        {
            BoundingBox leftBox, rightBox;
            int leftCount = 0, rightCount = 0;
            for (uint i = 0; i < node.primitiveCount; i++)
            {
                Primitive &primitive = primitives[primitiveIDs[node.leftFirst + i]];
                BoundingBox box;
                Primitive::boundingBox(primitive, box.min, box.max);
                if (Primitive::centroid(primitive)[axis] < pos)
                {
                    leftCount++;
                    leftBox.grow(box);
                }
                else
                {
                    rightCount++;
                    rightBox.grow(box);
                }
            }
            float cost = leftCount * leftBox.area() + rightCount * rightBox.area();
            return cost > 0 ? cost : FLT_MAX;
        }

        float calculateNodeCost(BvhNode &node)
        {
            return node.primitiveCount * node.box.area();
        }

        void subdivideSAH(uint nodeID, int num_intervals)
        {
            BvhNode &node = bvhNodes[nodeID];
            if (node.primitiveCount <= 2) return;

            int axis;
            float splitPos;
            float splitCost = findBestSplitPlaneSAH(node, axis, splitPos, num_intervals);

            float nosplitCost = calculateNodeCost(node);
            if (splitCost >= nosplitCost) return;

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
            
            subdivideSAH(leftChildID, num_intervals);
            subdivideSAH(rightChildID, num_intervals);
        }

        void subdivideBIN(uint nodeID, int num_intervals)
        {
            BvhNode &node = bvhNodes[nodeID];
            if (node.primitiveCount <= 2) return;

            int axis;
            float splitPos;
            float splitCost = findBestSplitPlaneBIN(node, axis, splitPos, num_intervals);

            float nosplitCost = calculateNodeCost(node);
            if (splitCost >= nosplitCost) return;

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
            
            subdivideBIN(leftChildID, num_intervals);
            subdivideBIN(rightChildID, num_intervals);
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

        void buildSAH(int num_intervals)
        {
            primitiveIDs.resize(primitives.size());
            for (int i = 0; i < primitives.size(); i++) primitiveIDs[i] = i;
            bvhNodes.resize(primitives.size() * 2);
            BvhNode &rootNode = bvhNodes[rootNodeID];
            rootNode.leftFirst = 0;
            rootNode.primitiveCount = primitives.size();
            updateNodeBound(rootNodeID);
            subdivideSAH(rootNodeID, num_intervals);
        }
        void buildBIN(int num_intervals)
        {
            primitiveIDs.resize(primitives.size());
            for (int i = 0; i < primitives.size(); i++) primitiveIDs[i] = i;
            bvhNodes.resize(primitives.size() * 2);
            BvhNode &rootNode = bvhNodes[rootNodeID];
            rootNode.leftFirst = 0;
            rootNode.primitiveCount = primitives.size();
            updateNodeBound(rootNodeID);
            subdivideBIN(rootNodeID, num_intervals);
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