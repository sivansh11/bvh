#ifndef BVH_HPP
#define BVH_HPP

#include "aabb.hpp"
#include "random.hpp"
#include "serilisation.hpp"

#include <memory>
#include <sstream>
#include <algorithm>
#include <queue>

namespace core {

namespace bvh {

struct node_t {
    static std::shared_ptr<node_t> create(const std::vector<uint32_t>& primitive_indices) {
        std::shared_ptr<node_t> node = std::make_shared<node_t>();
        node->aabb                          = null_aabb;
        node->parent                        = nullptr;
        node->left                          = nullptr;
        node->right                         = nullptr;
        node->primitive_indices             = primitive_indices;
        node->axis                          = 4;
        return node;
    }

    bool is_leaf() const {
        return primitive_indices.size() != 0 && (left == nullptr && right == nullptr);  // if node is leaf, primitive count will have a value and left and right will be nullptr
    }

    aabb_t                  aabb;
    std::shared_ptr<node_t> parent;
    std::shared_ptr<node_t> left;
    std::shared_ptr<node_t> right;
    uint32_t                axis;
    std::vector<uint32_t>   primitive_indices;
};

uint32_t node_depth(std::shared_ptr<node_t> node) {
    return node->is_leaf() ? 1 : 1 + max(node_depth(node->left), node_depth(node->right));
}

struct split_t {
    uint32_t axis;
    float    position;
};

static const split_t null_split { .axis = 4, .position = 0 };

enum class object_split_search_type_t {
    e_longest_axis_division,
    e_full_sweep_sah,
    e_uniform_sah,
    e_binned_sah,
};

struct bin_t {
    aabb_t   aabb;
    uint32_t primitive_count;
};

static const bin_t default_bin{ .aabb = null_aabb, .primitive_count = 0 };

struct bvh_t {
    uint32_t depth() const {
        return node_depth(root);
    }

    aabb_t root_aabb() {
        return root->aabb;
    }

    std::shared_ptr<node_t> root;
    uint32_t primitive_count;
};

struct builder_options_t {
    // options
    uint32_t                    _o_min_primitive_count        = 1;  // TODO: try 0
    uint32_t                    _o_max_primitive_count        = std::numeric_limits<uint32_t>::max();
    object_split_search_type_t  _o_object_split_search_type   = object_split_search_type_t::e_binned_sah;
    float                       _o_primitive_intersection_cost = 1.1f;
    float                       _o_node_intersection_cost     = 1.f;
    uint32_t                    _o_samples                    = 100;
};

struct builder_t {
    builder_t(const builder_options_t& builder_options)
      : _builder_options(builder_options) {}

    bvh_t build(const aabb_t *p_aabbs, const vec3 *p_centers, const uint32_t primitive_count) {
        _p_aabbs = p_aabbs;
        _p_centers = p_centers;
        _primitive_count = primitive_count;
   
        // create root node and start recursive building
        std::vector<uint32_t> primtiive_indices;
        for (uint32_t i = 0; i < _primitive_count; i++) primtiive_indices.push_back(i);
        auto root = node_t::create(primtiive_indices);
        update_node_bounds(root);
        try_split_node(root);

        bvh_t bvh;
        bvh.root              = root;
        bvh.primitive_count   = _primitive_count;        
        return bvh;
    }

    void update_node_bounds(std::shared_ptr<node_t> node) {
        for (uint32_t i = 0; i < node->primitive_indices.size(); i++) {
            const uint32_t primitive_id = node->primitive_indices[i];
            node->aabb.grow(_p_aabbs[primitive_id]);
        }
    }

    void try_split_node(std::shared_ptr<node_t> node) {
        if (node->primitive_indices.size() <= _builder_options._o_min_primitive_count) return;

        auto [split, split_cost] = find_best_object_split(node);
        
        if (node->primitive_indices.size() > _builder_options._o_max_primitive_count) {
            if (!split_node(node, split)) {
                // std::cout << "[WARNING] failed to split\n"; // maybe handle this ?
            }
        } else {
            float no_split_cost = cost_of_node(node);
            if (split_cost < no_split_cost) {
                if (!split_node(node, split)) {
                    // std::cout << "[WARNING] failed to split\n";
                }
            }
        }
    }

    float cost_of_node(std::shared_ptr<node_t> node) {
        if (node->is_leaf()) {
            return _builder_options._o_primitive_intersection_cost * node->primitive_indices.size();
        } else {
            return _builder_options._o_node_intersection_cost + ((node->left->aabb.area() * cost_of_node(node->left) + node->right->aabb.area() * cost_of_node(node->right)) / node->aabb.area());
        }
    }

    // assumes that the children are leafs
    float greedy_cost_of_node(uint32_t left_count, uint32_t right_count, float left_aabb_area, float right_aabb_area, const aabb_t& parent_aabb) {
        return _builder_options._o_node_intersection_cost + ((left_aabb_area * _builder_options._o_primitive_intersection_cost * left_count + right_aabb_area * _builder_options._o_primitive_intersection_cost * right_count) / parent_aabb.area());
    }

    float evaluate_sah(std::shared_ptr<node_t> node, const split_t& split) {
        aabb_t left_aabb = null_aabb;
        aabb_t right_aabb = null_aabb;
        uint32_t left_count = 0;
        uint32_t right_count = 0;
        for (uint32_t i = 0; i < node->primitive_indices.size(); i++) {
            const uint32_t primitive_id = node->primitive_indices[i];
            if (_p_centers[primitive_id][split.axis] < split.position) {
                left_aabb.grow(_p_aabbs[primitive_id]);
                left_count++;
            } else {
                right_aabb.grow(_p_aabbs[primitive_id]);
                right_count++;
            }
        }
        return greedy_cost_of_node(left_count, right_count, left_aabb.area(), right_aabb.area(), node->aabb);
    }

    // atm I only have object split, in future I would like to add spatial split also
    std::pair<split_t, float> find_best_object_split(std::shared_ptr<node_t> node) {
        split_t best_split = null_split;
        float   best_cost  = infinity;

        if (_builder_options._o_object_split_search_type == object_split_search_type_t::e_longest_axis_division) {
            vec3 e = node->aabb.max - node->aabb.min;
            best_split.axis = 0;
            if (e.y > e.x) best_split.axis = 1;
            if (e.z > e[best_split.axis]) best_split.axis = 2;
            best_split.position = node->aabb.min[best_split.axis] + e[best_split.axis] * 0.5f;
            best_cost = evaluate_sah(node, best_split);
        } else if (_builder_options._o_object_split_search_type == object_split_search_type_t::e_full_sweep_sah) {
            for (uint32_t axis = 0; axis < 3; axis++) {
                for (uint32_t i = 0; i < node->primitive_indices.size(); i++) {
                    const uint32_t primitive_id = node->primitive_indices[i];
                    split_t candidate_split;
                    candidate_split.axis = axis;
                    candidate_split.position = _p_centers[primitive_id][axis];
                    float cost = evaluate_sah(node, candidate_split);
                    if (cost < best_cost) {
                        best_cost = cost;
                        best_split = candidate_split;
                    }
                }
            }
        } else if (_builder_options._o_object_split_search_type == object_split_search_type_t::e_uniform_sah) {
            // this is smaller than the node's aabb as this is made from the centeroids of the primitives rather than the aabbs of the primitives.
            aabb_t split_bounds = null_aabb;
            for (uint32_t i = 0; i < node->primitive_indices.size(); i++) {
                const uint32_t primitive_id = node->primitive_indices[i];
                split_bounds.grow(_p_centers[primitive_id]);
            }

            for (uint32_t axis = 0; axis < 3; axis++) {
                const float scale = (split_bounds.max[axis] - split_bounds.min[axis]) / float(_builder_options._o_samples);
                if (scale == 0.f) continue;
                for (uint32_t i = 0; i < _builder_options._o_samples; i++) {
                    split_t candidate_split;
                    candidate_split.axis = axis;
                    candidate_split.position = split_bounds.min[axis] + (i * scale);
                    float cost = evaluate_sah(node, candidate_split);
                    if (cost < best_cost) {
                        best_cost = cost;
                        best_split = candidate_split;
                    }
                }
            }
        } else if (_builder_options._o_object_split_search_type == object_split_search_type_t::e_binned_sah) {
            // this is smaller than the node's aabb as this is made from the centeroids of the primitives rather than the aabbs of the primitives.
            aabb_t split_bounds = null_aabb;
            for (uint32_t i = 0; i < node->primitive_indices.size(); i++) {
                const uint32_t primitive_id = node->primitive_indices[i];
                split_bounds.grow(_p_centers[primitive_id]);
            }

            for (uint32_t axis = 0; axis < 3; axis++) {
                if (split_bounds.max[axis] == split_bounds.min[axis]) continue;
                
                bin_t bins[_builder_options._o_samples];
                for (uint32_t i = 0; i < _builder_options._o_samples; i++) {
                    bins[i] = default_bin;
                }

                float scale = static_cast<float>(_builder_options._o_samples) / (split_bounds.max[axis] - split_bounds.min[axis]);
                for (uint32_t i = 0; i < node->primitive_indices.size(); i++) {
                    const uint32_t primitive_id = node->primitive_indices[i];
                    uint32_t bin_id = min(_builder_options._o_samples - 1, static_cast<uint32_t>((_p_centers[primitive_id][axis] - split_bounds.min[axis]) * scale));
                    bins[bin_id].primitive_count++;
                    bins[bin_id].aabb.grow(_p_aabbs[primitive_id]);
                }

                float left_area[_builder_options._o_samples - 1], right_area[_builder_options._o_samples - 1];
                uint32_t left_count[_builder_options._o_samples - 1], right_count[_builder_options._o_samples - 1];
                aabb_t left_aabb = null_aabb, right_aabb = null_aabb;
                uint32_t left_sum = 0, right_sum = 0;
                for (uint32_t i = 0; i < _builder_options._o_samples - 1; i++) {
                    left_sum += bins[i].primitive_count;
                    left_count[i] = left_sum;
                    left_aabb.grow(bins[i].aabb);
                    left_area[i] = left_aabb.area();

                    right_sum += bins[_builder_options._o_samples - 1 - i].primitive_count;
                    right_count[_builder_options._o_samples - 2 - i] = right_sum;
                    right_aabb.grow(bins[_builder_options._o_samples - 1 - i].aabb);
                    right_area[_builder_options._o_samples - 2 - i] = right_aabb.area();
                }

                scale = (split_bounds.max[axis] - split_bounds.min[axis]) / static_cast<float>(_builder_options._o_samples);
                for (uint32_t i = 0; i < _builder_options._o_samples - 1; i++) {
                    float cost = greedy_cost_of_node(left_count[i], right_count[i], left_area[i], right_area[i], node->aabb);
                    if (cost < best_cost) {
                        best_cost = cost;
                        best_split = { .axis = axis, .position = split_bounds.min[axis] + scale * (i + 1) };
                    }
                }
            }
        }

        return { best_split, best_cost };
    }

    bool split_node(std::shared_ptr<node_t> node, const split_t& split) {
        if (split.axis == 4) {
            // std::cout << "[WARNING] invalid split\n";
            return false;
        }

        std::vector<uint32_t> left_primitive_indices;
        std::vector<uint32_t> right_primitive_indices;

        for (uint32_t i = 0; i < node->primitive_indices.size(); i++) {
            const uint32_t primitive_id = node->primitive_indices[i];
            if (_p_centers[primitive_id][split.axis] < split.position) {
                left_primitive_indices.push_back(primitive_id);
            } else {
                right_primitive_indices.push_back(primitive_id);
            }
        }

        uint32_t left_count = left_primitive_indices.size();
        if (left_count == 0 || left_count == node->primitive_indices.size()) {
            return false;  // failed split
        }

        node->left  = node_t::create(left_primitive_indices);
        node->right = node_t::create(right_primitive_indices);

        node->left->parent  = node;
        node->right->parent = node;
        node->left->axis = node->right->axis = split.axis;

        update_node_bounds(node->left);
        update_node_bounds(node->right);

        try_split_node(node->left);
        try_split_node(node->right);

        // maybe add is leaf bool and not clear this ?
        node->primitive_indices.clear(); 

        return true;  // succesfully split
    }

    float global_sah(const bvh_t& bvh) {
        return cost_of_node(bvh.root);
    }

    std::string show_info(const bvh_t& bvh) {
        std::stringstream s;

        uint32_t leaf_node_count = 0;
        uint32_t internal_node_count = 0;
        uint32_t node_count = 0;
        uint32_t min_primitives_per_leaf = std::numeric_limits<uint32_t>::max();
        uint32_t max_primitives_per_leaf = 0;
        float cost_of_tree = 0;
        uint32_t primitive_count = 0;
        std::vector<std::shared_ptr<core::bvh::node_t>> stack{ bvh.root };
        while (stack.size()) {
            auto node = stack[stack.size() - 1]; stack.pop_back();
            node_count++;

            if (node->is_leaf()) {
                leaf_node_count++;
                if (min_primitives_per_leaf > node->primitive_indices.size()) {
                    min_primitives_per_leaf = node->primitive_indices.size();
                }
                if (max_primitives_per_leaf < node->primitive_indices.size()) {
                    max_primitives_per_leaf = node->primitive_indices.size();
                }
                primitive_count += node->primitive_indices.size();
            } else {
                internal_node_count++;
                stack.push_back(node->left);
                stack.push_back(node->right);
            }
        }

        s << "bvh info\n";
        s << "\tdepth                                               :    " << bvh.depth() << '\n';    
        s << "\tnode count                                          :    " << node_count << '\n';
        // s << "\tprimitive count                                     :    " << bvh.primitive_count << '\n';
        s << "\tcounted primitive count (for testing only)          :    " << primitive_count << '\n';
        s << "\tleaf node count                                     :    " << leaf_node_count << '\n';    
        s << "\tinternal node count                                 :    " << internal_node_count << '\n';    
        s << "\tglobal sah cost                                     :    " << cost_of_node(bvh.root) << '\n';
        s << "\taverage primitives per leaf                         :    " << bvh.primitive_count / float(leaf_node_count) << '\n';
        s << "\tmin primitives per leaf                             :    " << min_primitives_per_leaf << '\n';
        s << "\tmax primitives per leaf                             :    " << max_primitives_per_leaf << '\n';

        return s.str();
    }

    // current inputs
    const aabb_t   *_p_aabbs;
    const vec3     *_p_centers;
    uint32_t  _primitive_count;

    // options
    const builder_options_t _builder_options; 
};

} // namespace bvh

} // namespace core 

#endif