#ifndef BVH_HPP
#define BVH_HPP

#include "aabb.hpp"
#include "random.hpp"

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

    std::shared_ptr<node_t> root;
    uint32_t primitive_count;
};

struct builder_t {
    builder_t(const aabb_t *p_aabbs, const vec3 *p_centers, const uint32_t primitive_count)
      : _p_aabbs(p_aabbs), _p_centers(p_centers), _primitive_count(primitive_count) {}

    builder_t& set_min_primitive_count(uint32_t min_primitive_count) {
        _o_min_primitive_count = min_primitive_count;
        return *this;
    }

    builder_t& set_max_primitive_count(uint32_t max_primitive_count) {
        _o_max_primitive_count = max_primitive_count;
        return *this;
    }

    builder_t& set_object_split_search_type(object_split_search_type_t object_split_search_type) {
        _o_object_split_search_type = object_split_search_type;
        return *this;
    }

    builder_t& set_triangle_intersection_cost(float triangle_intersection_cost) {
        _o_triangle_intersection_cost = triangle_intersection_cost;
        return *this;
    }

    builder_t& set_node_intersection_cost(float node_intersection_cost) {
        _o_node_intersection_cost = node_intersection_cost;
        return *this;
    }

    builder_t& set_samples(uint32_t samples) {
        _o_samples = samples;
        return *this;
    }

    bvh_t build() {
   
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
        if (node->primitive_indices.size() <= _o_min_primitive_count) return;

        auto [split, split_cost] = find_best_object_split(node);
        
        if (node->primitive_indices.size() > _o_max_primitive_count) {
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
            return _o_triangle_intersection_cost * node->primitive_indices.size();
        } else {
            return _o_node_intersection_cost + ((node->left->aabb.area() * cost_of_node(node->left) + node->right->aabb.area() * cost_of_node(node->right)) / node->aabb.area());
        }
    }

    // assumes that the children are leafs
    float greedy_cost_of_node(uint32_t left_count, uint32_t right_count, float left_aabb_area, float right_aabb_area, const aabb_t& parent_aabb) {
        return _o_node_intersection_cost + ((left_aabb_area * _o_triangle_intersection_cost * left_count + right_aabb_area * _o_triangle_intersection_cost * right_count) / parent_aabb.area());
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

        if (_o_object_split_search_type == object_split_search_type_t::e_longest_axis_division) {
            vec3 e = node->aabb.max - node->aabb.min;
            best_split.axis = 0;
            if (e.y > e.x) best_split.axis = 1;
            if (e.z > e[best_split.axis]) best_split.axis = 2;
            best_split.position = node->aabb.min[best_split.axis] + e[best_split.axis] * 0.5f;
            best_cost = evaluate_sah(node, best_split);
        } else if (_o_object_split_search_type == object_split_search_type_t::e_full_sweep_sah) {
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
        } else if (_o_object_split_search_type == object_split_search_type_t::e_uniform_sah) {
            // this is smaller than the node's aabb as this is made from the centeroids of the primitives rather than the aabbs of the primitives.
            aabb_t split_bounds = null_aabb;
            for (uint32_t i = 0; i < node->primitive_indices.size(); i++) {
                const uint32_t primitive_id = node->primitive_indices[i];
                split_bounds.grow(_p_centers[primitive_id]);
            }

            for (uint32_t axis = 0; axis < 3; axis++) {
                const float scale = (split_bounds.max[axis] - split_bounds.min[axis]) / float(_o_samples);
                if (scale == 0.f) continue;
                for (uint32_t i = 0; i < _o_samples; i++) {
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
        } else if (_o_object_split_search_type == object_split_search_type_t::e_binned_sah) {
            // this is smaller than the node's aabb as this is made from the centeroids of the primitives rather than the aabbs of the primitives.
            aabb_t split_bounds = null_aabb;
            for (uint32_t i = 0; i < node->primitive_indices.size(); i++) {
                const uint32_t primitive_id = node->primitive_indices[i];
                split_bounds.grow(_p_centers[primitive_id]);
            }

            for (uint32_t axis = 0; axis < 3; axis++) {
                if (split_bounds.max[axis] == split_bounds.min[axis]) continue;
                
                bin_t bins[_o_samples];
                for (uint32_t i = 0; i < _o_samples; i++) {
                    bins[i] = default_bin;
                }

                float scale = static_cast<float>(_o_samples) / (split_bounds.max[axis] - split_bounds.min[axis]);
                for (uint32_t i = 0; i < node->primitive_indices.size(); i++) {
                    const uint32_t primitive_id = node->primitive_indices[i];
                    uint32_t bin_id = min(_o_samples - 1, static_cast<uint32_t>((_p_centers[primitive_id][axis] - split_bounds.min[axis]) * scale));
                    bins[bin_id].primitive_count++;
                    bins[bin_id].aabb.grow(_p_aabbs[primitive_id]);
                }

                float left_area[_o_samples - 1], right_area[_o_samples - 1];
                uint32_t left_count[_o_samples - 1], right_count[_o_samples - 1];
                aabb_t left_aabb = null_aabb, right_aabb = null_aabb;
                uint32_t left_sum = 0, right_sum = 0;
                for (uint32_t i = 0; i < _o_samples - 1; i++) {
                    left_sum += bins[i].primitive_count;
                    left_count[i] = left_sum;
                    left_aabb.grow(bins[i].aabb);
                    left_area[i] = left_aabb.area();

                    right_sum += bins[_o_samples - 1 - i].primitive_count;
                    right_count[_o_samples - 2 - i] = right_sum;
                    right_aabb.grow(bins[_o_samples - 1 - i].aabb);
                    right_area[_o_samples - 2 - i] = right_aabb.area();
                }

                scale = (split_bounds.max[axis] - split_bounds.min[axis]) / static_cast<float>(_o_samples);
                for (uint32_t i = 0; i < _o_samples - 1; i++) {
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

    // inputs
    const aabb_t   *_p_aabbs;
    const vec3     *_p_centers;
    const uint32_t  _primitive_count;

    // options
    uint32_t                    _o_min_primitive_count        = 1;  // TODO: try 0
    uint32_t                    _o_max_primitive_count        = std::numeric_limits<uint32_t>::max();
    object_split_search_type_t  _o_object_split_search_type   = object_split_search_type_t::e_binned_sah;
    float                       _o_triangle_intersection_cost = 1.1f;
    float                       _o_node_intersection_cost     = 1.f;
    uint32_t                    _o_samples                    = 100;
};

struct flat_node_t {
    bool is_leaf() const { return primitive_count != 0; }
    aabb_t aabb;
    uint32_t first_index = 0;
    uint32_t primitive_count = 0;
};

struct flat_bvh_t {
    std::vector<flat_node_t> flat_nodes;
    std::vector<uint32_t> primitive_indices;
    uint32_t primitive_count;
};

struct post_process_t {
    post_process_t(bvh_t& bvh, const builder_t& builder, uint32_t seed = 0) : _bvh(bvh), _builder(builder), _rng(seed) {}

    float inefficiency_measure_sum(std::shared_ptr<node_t> node) {
        return node->aabb.area() / (0.5 * (node->left->aabb.area() + node->right->aabb.area()));
    }

    float inefficiency_measure_min(std::shared_ptr<node_t> node) {
        return node->aabb.area() / (min(node->left->aabb.area(), node->right->aabb.area()));
    }

    float inefficiency_measure_area(std::shared_ptr<node_t> node) {
        return node->aabb.area();
    }

    float inefficiency_measure(std::shared_ptr<node_t> node) {
        return inefficiency_measure_sum(node) * inefficiency_measure_min(node) * inefficiency_measure_area(node);
    }

    post_process_t& reinsertion_optimization(uint32_t k, uint32_t itrs) {
        for (uint32_t pass = 0; pass < itrs; pass++) {
            auto candidates = find_candidates(_bvh.root);
            std::sort(candidates.begin(), candidates.end(), [&](std::shared_ptr<node_t> a, std::shared_ptr<node_t> b) {
                return inefficiency_measure(a) > inefficiency_measure(b);
            });
            for (uint32_t i = 0; i < min(k, static_cast<uint32_t>(candidates.size())); i++) {
                std::shared_ptr<node_t> from;
                from = candidates[i];
                auto remove = remove_node(from);
                {
                    auto to = search_for_reinsertion_node(_bvh.root, remove.children[0]);
                    reinsert(to, remove.children[0], remove.free[0]);
                }
                {
                    auto to = search_for_reinsertion_node(_bvh.root, remove.children[1]);
                    reinsert(to, remove.children[1], remove.free[1]);
                }
            }
        }
        return *this;
    }

    float cost_of_node(std::shared_ptr<node_t> node) {
        if (node->is_leaf()) {
            return _builder._o_triangle_intersection_cost * node->primitive_indices.size();
        } else {
            return _builder._o_node_intersection_cost + ((node->left->aabb.area() * cost_of_node(node->left) + node->right->aabb.area() * cost_of_node(node->right)) / node->aabb.area());
        }
    }

    void post_order_traversal_collapse_unnecessary_subtrees(std::shared_ptr<node_t> node) {
        if (!node->is_leaf()) {
            post_order_traversal_collapse_unnecessary_subtrees(node->left);
            post_order_traversal_collapse_unnecessary_subtrees(node->right);
        } 

        if (!node->is_leaf() && node->left->is_leaf() && node->right->is_leaf()) {
            std::vector<std::shared_ptr<node_t>> stack{ node };

            std::vector<uint32_t> primitive_indices;
            while (stack.size()) {
                auto current = stack.back(); stack.pop_back();
                if (current->is_leaf()) {
                    for (uint32_t i = 0; i < current->primitive_indices.size(); i++) {
                        const uint32_t primitive_id = current->primitive_indices[i];
                        primitive_indices.push_back(primitive_id);
                    }
                } else {
                    stack.push_back(current->left);
                    stack.push_back(current->right);
                }
            }
            float real_cost_of_node = cost_of_node(node);
            float cost_of_node_if_leaf = _builder._o_triangle_intersection_cost * primitive_indices.size();

            if (cost_of_node_if_leaf < real_cost_of_node) {
                node->primitive_indices = primitive_indices;
                node->left->parent = nullptr;
                node->right->parent = nullptr;
                node->left = nullptr;
                node->right = nullptr;
            }
        }
    }

    post_process_t& collapse_unnecessary_subtrees() {
        post_order_traversal_collapse_unnecessary_subtrees(_bvh.root);
        return *this;
    }

    std::shared_ptr<node_t> get_sibling(std::shared_ptr<node_t> node) {
        auto parent = node->parent;
        return parent->left == node ? parent->right : parent->left;
    }

    void recompute_node_bounds_from(std::shared_ptr<node_t> node) {
        do {
            node->aabb = null_aabb;
            node->aabb.grow(node->left->aabb).grow(node->right->aabb);
            node = node->parent;
        } while (node != nullptr);
        // do {
        //     node = node->parent;
        // } while (node->parent != nullptr);

        // std::vector<std::shared_ptr<node_t>> stack { node };
        // std::vector<std::shared_ptr<node_t>> update_order;
        // while (stack.size()) {
        //     auto current = stack.back(); stack.pop_back();
        //     update_order.push_back(current);
        //     if (current->is_leaf()) {
        //         stack.push_back(current->left);
        //         stack.push_back(current->right);
        //     }
        // }

        // for (int i = update_order.size() - 1; i >= 0; i--) {
        //     auto current = update_order[i];
        //     if (current->is_leaf()) continue;
        //     current->aabb = null_aabb;
        //     current->aabb.grow(current->left->aabb).grow(current->right->aabb);
        // }
    }

    struct remove_t {
        std::shared_ptr<node_t> children[2];
        std::shared_ptr<node_t> free[2];
    };

    remove_t remove_node(std::shared_ptr<node_t> node) {
        auto parent = node->parent;
        auto left = node->left;
        auto right = node->right;
        auto sibling = get_sibling(node);
        auto grandparent = parent->parent;

        if (node->is_leaf()) {
            std::cout << "[WARNING] Tried to remove leaf node, its not allowed\n";
            return {};
        }

        // remove node parent from the tree, this effectively removes the sub branch that contains node
        if (grandparent->left == parent) {
            grandparent->left = sibling;
        } else {
            grandparent->right = sibling;
        }
        sibling->parent = grandparent;

        // removing refrences so that shared pointer can update internal refrence counts
        parent->parent = nullptr;  // remove refrence of grandparent from parent
        node->left = nullptr;  // remove refrence of children
        node->right = nullptr;
        left->parent = nullptr;     // children should also not refer to node
        right->parent = nullptr;

        // update tree bounds
        recompute_node_bounds_from(grandparent);
        
        if (left->aabb.area() > right->aabb.area()) {
            return {
                { left, right },
                { node, parent },
            };
        } else {
            return {
                { right, left },
                { node, parent },
            };
        }
    }

    float calculate_direct_cost(std::shared_ptr<node_t> node, std::shared_ptr<node_t> candidate) {
        return aabb_t{ null_aabb }.grow(node->aabb).grow(candidate->aabb).area();
    }

    float calculate_induced_cost(std::shared_ptr<node_t> node, std::shared_ptr<node_t> candidate) {
        float induced_cost = 0;
        while (candidate->parent) {
            candidate = candidate->parent;
            induced_cost += aabb_t{ null_aabb }.grow(candidate->aabb).grow(node->aabb).area() - candidate->aabb.area();
        }
        return induced_cost;
    }

    std::shared_ptr<node_t> search_for_reinsertion_node(std::shared_ptr<node_t> root, std::shared_ptr<node_t> node) {
        // TODO: use priority queue

        std::shared_ptr<node_t> best_candidate = nullptr;
        float best_cost = infinity;

        std::vector<std::shared_ptr<node_t>> stack{ root };
        while (stack.size()) {
            auto candidate = stack.back(); stack.pop_back();
            float direct_cost = calculate_direct_cost(node, candidate);
            float induced_cost = calculate_induced_cost(node, candidate);
            float cost = direct_cost + induced_cost;
            if (cost < best_cost && candidate != root) {
                best_cost = cost;
                best_candidate = candidate;
            }
            if (!candidate->is_leaf()) {
                stack.push_back(candidate->left);
                stack.push_back(candidate->right);
            }
        }
        return best_candidate;
    }

    void reinsert(std::shared_ptr<node_t> to, std::shared_ptr<node_t> from, std::shared_ptr<node_t> link) {
        auto to_parent = to->parent;

        if (to_parent->left == to) {
            to_parent->left = link;
            link->parent = to_parent;
        } else {
            to_parent->right = link;
            link->parent = to_parent;
        }

        link->left = to;
        link->right = from;

        to->parent = link;
        from->parent = link;

        recompute_node_bounds_from(link);
    }

    std::vector<std::shared_ptr<node_t>> find_candidates(std::shared_ptr<node_t> root) {
        std::vector<std::shared_ptr<node_t>> nodes;
        std::vector<std::pair<std::shared_ptr<node_t>, uint32_t>> stack { { root, 1 } };
        while (stack.size()) {
            auto [node, depth] = stack.back(); stack.pop_back();
            if (!node->is_leaf()) {  // leaf nodes cant be candidates cause inefficiency measure requires left and right children
                if (depth > 3) nodes.push_back(node);
                stack.push_back({ node->left, depth + 1 });
                stack.push_back({ node->right, depth + 1 });
            }
        }
        return nodes;
    }

    flat_bvh_t flatten() {
        // TODO: add flattening, check if node collapse works after flattening ?
        std::vector<flat_node_t> flat_nodes;
        std::deque<std::pair<std::shared_ptr<node_t>, uint32_t>> deque{ { _bvh.root, std::numeric_limits<uint32_t>::max() } };

        std::vector<uint32_t> primitive_indices;
        uint32_t primitive_offset = 0;

        bool is_left = false;
        while (deque.size()) {
            auto [current, parent_id] = deque.front(); deque.pop_front();
            flat_node_t flat_node{};
            flat_node.aabb = current->aabb;
            flat_node.first_index = 0;
            flat_node.primitive_count = 0;
            flat_nodes.push_back(flat_node);
            if (is_left) {
                flat_nodes[parent_id].first_index = flat_nodes.size() - 1;
            } 
            is_left = !is_left;

            if (!current->is_leaf()) {
                deque.push_back({ current->left, flat_nodes.size() - 1});
                deque.push_back({ current->right, flat_nodes.size() - 1});
            } else {
                // weird hack, basically just need to set the first index and primitive count of the current flat node which is already in the array
                flat_nodes.back().first_index = primitive_offset;
                flat_nodes.back().primitive_count = current->primitive_indices.size();
                for (uint32_t i = 0; i < current->primitive_indices.size(); i++) {
                    const uint32_t primitive_id = current->primitive_indices[i];
                    primitive_indices.push_back(primitive_id);
                }
                primitive_offset += current->primitive_indices.size();
            }
        }
        flat_bvh_t flat_bvh;
        flat_bvh.primitive_count = _bvh.primitive_count;
        flat_bvh.primitive_indices = primitive_indices;
        flat_bvh.flat_nodes = flat_nodes;
        return flat_bvh;
    }

    bvh_t& _bvh;
    const builder_t& _builder;
    rng_t _rng;
};

} // namespace bvh

} // namespace core 

#endif