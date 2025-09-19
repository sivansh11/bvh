#include "bvh/bvh.hpp"

#include <vector>

#include "math/aabb.hpp"
#include "math/math.hpp"

namespace bvh {

uint64_t split(uint64_t x, int log_bits) {
  const int bit_count = 1 << log_bits;
  uint64_t  mask      = ((uint64_t)-1) >> (bit_count / 2);
  x &= mask;
  for (int i = log_bits - 1, n = 1 << i; i > 0; --i, n >>= 1) {
    mask = (mask | (mask << n)) & ~(mask << (n / 2));
    x    = (x | (x << n)) & mask;
  }
  return x;
}

uint64_t encode(uint64_t x, uint64_t y, uint64_t z, int log_bits) {
  return split(x, log_bits) | (split(y, log_bits) << 1) |
         (split(z, log_bits) << 2);
}

uint32_t find_best_node(const bvh_t& bvh, uint32_t node_index,
                        uint32_t search_radius) {
  const node_t&  node       = bvh.nodes[node_index];
  uint32_t       best_index = node_index;
  float          best_area  = math::infinity;
  const uint32_t node_count = bvh.nodes.size();
  uint32_t start = node_index > search_radius ? node_index - search_radius : 0;
  uint32_t stop  = node_index + search_radius < node_count
                       ? node_index + search_radius
                       : node_count - 1;
  for (uint32_t index = start; index < stop; index++) {
    float area = node.aabb().grow(bvh.nodes[index].aabb()).area();
    if (area < best_area) {
      best_area  = area;
      best_index = index;
    }
  }
  return best_index;
}

bvh_t build_bvh(const model::raw_mesh_t& mesh) {
  bvh_t bvh{};

  for (uint32_t i = 0; i < mesh.indices.size(); i += 3) {
    bvh_triangle_t bvh_triangle{
        {mesh.vertices[mesh.indices[i + 0]].position, 0},
        {mesh.vertices[mesh.indices[i + 1]].position, 0},
        {mesh.vertices[mesh.indices[i + 2]].position, 0},
    };
    bvh.triangles.push_back(bvh_triangle);
  }

  bvh.nodes = std::vector<node_t>(2 * bvh.triangles.size() - 1);

  uint32_t start = bvh.nodes.size() - bvh.triangles.size();
  uint32_t end   = bvh.nodes.size();

  while (true) {
  }

  return bvh;
}

}  // namespace bvh
