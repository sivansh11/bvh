#ifndef POSTPROCESSING_HPP
#define POSTPROCESSING_HPP

#include "aabb.hpp"
#include "bvh.hpp"

#include <vector>
#include <deque>
#include <iostream>

namespace core {

namespace bvh {

struct flat_node_t {
    bool is_leaf() const { return primitive_count != 0; }
    aabb_t aabb;
    uint32_t first_index = 0;
    uint32_t primitive_count = 0;
};

struct flat_bvh_t {
    aabb_t root_aabb() {
        return flat_nodes[0].aabb;
    }
    std::vector<flat_node_t> flat_nodes;
    std::vector<uint32_t> primitive_indices;
};


// TODO: post processing options ?

struct post_processing_t {
    post_processing_t(const builder_options_t& builder_options) : _builder_options(builder_options) {}

    post_processing_t& reinsertion_optimization(bvh_t& bvh, uint32_t itrs) {
        for (uint32_t pass = 0; pass < itrs; pass++) {
            
            auto candidates = find_candidates(bvh.root);
            std::sort(candidates.begin(), candidates.end(), [&](std::shared_ptr<node_t> a, std::shared_ptr<node_t> b) {
                return inefficiency_measure(a) > inefficiency_measure(b);
            });
            for (uint32_t i = 0; i < candidates.size() / 100; i++) {
                std::stringstream s;
                s << "at pass " << pass << " iteration " << i << " out of " << candidates.size() / 100;
                std::cout << s.str();
                std::cout << std::string(s.str().size(), '\b');
                std::cout.flush();
                std::shared_ptr<node_t> from;
                from = candidates[i];
                if (!from->parent || !from->parent->parent) continue;
                auto remove = remove_node(from);
                {
                    auto to = search_for_reinsertion_node(bvh.root, remove.children[0]);
                    reinsert(to, remove.children[0], remove.free[0]);
                }
                {
                    auto to = search_for_reinsertion_node(bvh.root, remove.children[1]);
                    reinsert(to, remove.children[1], remove.free[1]);
                }
            }
        }
        return *this;
    }

    post_processing_t& node_collapse_optimization(bvh_t& bvh) {
        // std::cout << "[INFO] starting collapse optimisation!\n";
        uint32_t nodes_collapsed = 0;
        post_order_traversal_node_collapse_optimization(bvh.root, nodes_collapsed);
        std::cout << "[INFO] collapsed " << nodes_collapsed << " nodes!\n";
        return *this;
    }

    flat_bvh_t flatten(bvh_t& bvh) {
        std::vector<flat_node_t> flat_nodes;
        std::deque<std::pair<std::shared_ptr<node_t>, uint32_t>> deque{ { bvh.root, std::numeric_limits<uint32_t>::max() } };

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
        flat_bvh.primitive_indices = primitive_indices;
        flat_bvh.flat_nodes = flat_nodes;
        return flat_bvh;
    }

private:
    float inefficiency_measure_sum(std::shared_ptr<node_t> node) {
        return node->aabb.area() / (0.5f * (node->left->aabb.area() + node->right->aabb.area()));
    }

    float inefficiency_measure_min(std::shared_ptr<node_t> node) {
        return node->aabb.area() / (min(node->left->aabb.area(), node->right->aabb.area()));
    }

    float inefficiency_measure_area(std::shared_ptr<node_t> node) {
        return node->aabb.area();
    }

    float inefficiency_measure(std::shared_ptr<node_t> node) {
        float inefficiency = inefficiency_measure_sum(node) * inefficiency_measure_min(node) * inefficiency_measure_area(node);
        if (glm::isinf(inefficiency)) {
            return 0;
        }
        return inefficiency;
    }


    float cost_of_node(std::shared_ptr<node_t> node) {
        if (node->is_leaf()) {
            return _builder_options._o_primitive_intersection_cost * node->primitive_indices.size();
        } else {
            return _builder_options._o_node_intersection_cost + ((node->left->aabb.area() * cost_of_node(node->left) + node->right->aabb.area() * cost_of_node(node->right)) / node->aabb.area());
        }
    }

    void post_order_traversal_node_collapse_optimization(std::shared_ptr<node_t> node, uint32_t& nodes_collapsed) {
        if (!node->is_leaf()) {
            post_order_traversal_node_collapse_optimization(node->left, nodes_collapsed);
            post_order_traversal_node_collapse_optimization(node->right, nodes_collapsed);
        } 

        if (!node->is_leaf()) {
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
            float cost_of_node_if_leaf = _builder_options._o_primitive_intersection_cost * primitive_indices.size();

            if (cost_of_node_if_leaf < real_cost_of_node) {
                nodes_collapsed++;
                // std::cout << "[INFO] collapsed a subtree!\n";
                node->primitive_indices = primitive_indices;
                node->left->parent = nullptr;
                node->right->parent = nullptr;
                node->left = nullptr;
                node->right = nullptr;
            }
        }
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
        node->parent = nullptr;
        node->left = nullptr;  // remove refrence of children
        node->right = nullptr;
        left->parent = nullptr;     // children should also not refer to node
        right->parent = nullptr;

        // update tree bounds
        recompute_node_bounds_from(grandparent);

        grandparent->is_leaf();
        sibling->is_leaf();
        
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
        while (true) {
            if (candidate->parent == nullptr) return induced_cost;
            induced_cost += aabb_t{ null_aabb }.grow(candidate->parent->aabb).grow(node->aabb).area() - candidate->parent->aabb.area();
            candidate = candidate->parent;
        }
    }

    struct priority_element_t {
        std::shared_ptr<node_t> node;
        float priority;
        bool operator < (const priority_element_t& other) const {
            return priority > other.priority; // > cause inversely proportional
        }
    };

    std::shared_ptr<node_t> search_for_reinsertion_node(std::shared_ptr<node_t> root, std::shared_ptr<node_t> node) {
        // TODO: use priority queue

        std::shared_ptr<node_t> best_candidate = nullptr;
        float best_cost = infinity;

        std::priority_queue<priority_element_t> pq;
        pq.push({root, 0});

        while (pq.size()) {
            auto [candidate, priority] = pq.top();
            pq.pop();

            float direct_cost = calculate_direct_cost(node, candidate);
            float induced_cost = calculate_induced_cost(node, candidate);

            if (induced_cost + node->aabb.area() >= best_cost) break;

            float cost = direct_cost + induced_cost;

            if (cost < best_cost) {
                best_cost = cost;
                best_candidate = candidate;
            }

            float induced_cost_children = cost - candidate->aabb.area();

            if ((induced_cost_children + node->aabb.area()) < best_cost) {
                if (!candidate->is_leaf()) {
                    pq.push({candidate->left, induced_cost_children});
                    pq.push({candidate->right, induced_cost_children});
                }
            }
        }
        return best_candidate;
    }

    void reinsert(std::shared_ptr<node_t> to, std::shared_ptr<node_t> from, std::shared_ptr<node_t> link) {
        auto to_parent = to->parent;

        if (to_parent) {
            if (to_parent->left == to) {
                to_parent->left = link;
                link->parent = to_parent;
            } else {
                to_parent->right = link;
                link->parent = to_parent;
            }
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

    const builder_options_t& _builder_options;
};

void to_disk(const flat_bvh_t& bvh, std::string path) {
    binary_writer_t binary_writer{ path };
    binary_writer.write(static_cast<uint32_t>(bvh.flat_nodes.size()));
    for (uint32_t i = 0; i < bvh.flat_nodes.size(); i++) {
        binary_writer.write(bvh.flat_nodes[i]);
    }
    binary_writer.write(static_cast<uint32_t>(bvh.primitive_indices.size()));
    for (uint32_t i = 0; i < bvh.primitive_indices.size(); i++) {
        binary_writer.write(bvh.primitive_indices[i]);
    }
}

flat_bvh_t load(std::string path) {
    flat_bvh_t bvh{};
    binary_reader_t binary_reader{ path };
    uint32_t node_count;
    binary_reader.read(node_count);
    for (uint32_t i = 0; i < node_count; i++) {
        flat_node_t node;
        binary_reader.read(node);
        bvh.flat_nodes.push_back(node);
    }
    uint32_t primitive_count;
    binary_reader.read(primitive_count);
    for (uint32_t i = 0; i < primitive_count; i++) {
        uint32_t primitive_id;
        binary_reader.read(primitive_id);
        bvh.primitive_indices.push_back(primitive_id);
    }
    return bvh;
}


} // namespace bvh

} // namespace core

#endif