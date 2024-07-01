#ifndef CORE_ECS_HPP
#define CORE_ECS_HPP

#include <iostream>
#include <limits>
#include <cstdint>
#include <utility>
#include <vector>

#define my_assert(truthy, msg) \
do { \
    if (!(truthy)) { \
        std::cout << msg; \
        std::terminate(); \
    } \
} while(false)

namespace core {

// storage would be u32/u64 or something similar, its the underlaying storage of the bits
template <typename storage_t>
class bit_set_t {
public: 
    bit_set_t(storage_t data = 0) : _data(data) {}

    void set(uint32_t n) {
        my_assert(n < (sizeof(storage_t) * 8), ".");
        _data = _data | (static_cast<storage_t>(1) << n);
    }

    void unset(uint32_t n) {
        my_assert(n < (sizeof(storage_t) * 8), ".");
        _data = _data & ~(static_cast<storage_t>(1) << n);
    }

    void toggle(uint32_t n) {
        my_assert(n < (sizeof(storage_t) * 8), ".");
        _data = _data ^ (static_cast<storage_t>(1) << n);
    }

    bool test(uint32_t n) const {
        my_assert(n < (sizeof(storage_t) * 8), ".");
        return (_data & (static_cast<storage_t>(1) << n)) != 0;
    }

    bool test_all(const bit_set_t& other) const {
        return (other._data & ~_data) == 0;
    }

    constexpr uint32_t size() const {
        return sizeof(storage_t) * 8;
    }

private:
    storage_t _data;
};

template <typename storage_t>
std::ostream& operator<<(std::ostream& o, const core::bit_set_t<storage_t>& bit_set) {
    for (uint32_t i = 0; i < sizeof(storage_t) * 8; i++) {
        o << bit_set.test(i);
    }
    return o;
}

using entity_id_t = uint32_t;
using component_id_t = uint32_t;
using component_mask_t = bit_set_t<uint32_t>;

static const entity_id_t null_entity_id = std::numeric_limits<entity_id_t>::max();
static component_id_t component_id_counter = 0;

template <typename T>
inline component_id_t get_component_id_for() {
    static component_id_t s_component_id = component_id_counter++;
    return s_component_id;
}

struct base_component_pool_t {
    ~base_component_pool_t() {
        delete[] reinterpret_cast<char *>(_p_data);
    }

    void *get(entity_id_t index) {
        my_assert(index < _max_entities, "index is out of bounds");
        my_assert(_p_data, "component pool was not created ?");
        return reinterpret_cast<char *>(_p_data) + index * _component_size;
    }

    virtual void destroy(entity_id_t index) = 0;

    component_id_t          _id;
    uint32_t                _component_size;
    entity_id_t             _max_entities;
    void                   *_p_data{ nullptr };
};

template <typename T>
struct component_pool_t : public base_component_pool_t {
    component_pool_t(entity_id_t max_entities) {
        _component_size = sizeof(T);
        _max_entities = max_entities;
        _p_data = new char[_component_size * _max_entities];
        _id = get_component_id_for<T>();
    }

    ~component_pool_t() {}

    template <typename... args_t>
    T *construct(entity_id_t index, args_t&&... args) {
        T *component = reinterpret_cast<T *>(get(index));
        new (component) T{ std::forward<args_t>(args)... };
        return component;
    }

    void destroy(entity_id_t index) override {
        T *component = reinterpret_cast<T *>(get(index));
        component->~T();
    }
};

class scene_t {
    struct entity_description_t {
        entity_id_t         id;
        component_mask_t    mask;
        bool                is_valid;
    };

public:
    scene_t(entity_id_t max_entities = 1000) : _max_entities(max_entities) {
        _available_entities.reserve(_max_entities);
        entity_id_t counter = _max_entities;
        while (counter--) {
            _available_entities.push_back(counter);
        }
    }

    ~scene_t() {
        for (auto& entity : _entities) if (entity.is_valid) {
            for (auto& component_pool : _component_pools) if (entity.mask.test(component_pool->_id)) {
                component_pool->destroy(entity.id);
            }
        }

        for (auto& component_pool : _component_pools) {
            delete component_pool;
        }
    }

    entity_id_t create() {
        my_assert(_available_entities.size() > 0, "entity limit reached!");
        entity_id_t id = _available_entities[_available_entities.size() - 1];
        _available_entities.pop_back();
        if (_entities.size() <= id) {
            entity_description_t entity_description = {
                .id         = id,
                .mask       = {},
                .is_valid   = true
            };
            _entities.push_back(entity_description);
        } else {
            my_assert(!_entities[id].is_valid, "new entity with id " << id << " already in used, yet it is in the available entities stack");
        }
        return id;
    }

    void destroy(entity_id_t id) {
        my_assert(id != null_entity_id, "cant destroy null entity");
        my_assert(_entities[id].is_valid, "cant destroy a destroyed entity");
        entity_description_t& entity_description = _entities[id];
        for (component_id_t i = 0; i < entity_description.mask.size(); i++) {
            if (entity_description.mask.test(i)) {
                _component_pools[i]->destroy(id);
            }
        }
        entity_description.is_valid = false;
        entity_description.mask     = {};
        _available_entities.push_back(id);
    }
    
    template <typename T>
    T& get(entity_id_t id) {
        my_assert(id != null_entity_id, "cant get from null entity");
        my_assert(_entities[id].is_valid, "cant get from deleted entity");

        component_id_t component_id = get_component_id_for<T>();
        if (_component_pools.size() <= component_id) {
            _component_pools.resize(component_id + 1);
            _component_pools[component_id] = new component_pool_t<T>(_max_entities);
        }

        entity_description_t& entity_description = _entities[id];
        entity_description.mask.set(component_id);
        return *reinterpret_cast<T *>(_component_pools[component_id]->get(id));
    }

    template <typename T, typename... args_t>
    T& construct(entity_id_t id, args_t&&... args) {
        my_assert(id != null_entity_id, "cant get from null entity");
        my_assert(_entities[id].is_valid, "cant get from deleted entity");
        my_assert(!_entities[id].mask.test(get_component_id_for<T>()), "entity already has component");


        component_id_t component_id = get_component_id_for<T>();
        if (_component_pools.size() <= component_id) {
            _component_pools.resize(component_id + 1);
            _component_pools[component_id] = new component_pool_t<T>(_max_entities);
        }

        entity_description_t& entity_description = _entities[id];
        entity_description.mask.set(component_id);
        return *(reinterpret_cast<component_pool_t<T> *>(_component_pools[component_id])->construct(id, std::forward<args_t>(args)...));
    }

    template <typename T>
    void remove(entity_id_t id) {
        my_assert(id != null_entity_id, "cant get from null entity");
        my_assert(_entities[id].is_valid, "cant get from deleted entity");
        my_assert(_entities[id].mask.test(get_component_id_for<T>()), "entity doesnt have component");

        component_id_t component_id = get_component_id_for<T>();
        if (_component_pools.size() <= component_id) {
            _component_pools.resize(component_id + 1);
            _component_pools[component_id] = new component_pool_t<T>(_max_entities);
        }

        _component_pools[component_id]->destroy(id);

        entity_description_t& entity_description = _entities[id];
        entity_description.mask.unset(component_id);   
    }

    template <typename... T>
    bool has(entity_id_t id) {
        my_assert(id != null_entity_id, "cant get from null entity");
        my_assert(_entities[id].is_valid, "cant get from deleted entity");

        component_mask_t mask{};
        component_id_t component_ids[] = { get_component_id_for<T>()... };
        for (uint32_t i = 0; i < sizeof...(T); i++) {
            mask.set(component_ids[i]);
        }

        entity_description_t& entity_description = _entities[id];
        std::cout << mask << '\n';
        std::cout << entity_description.mask << '\n';

        return entity_description.mask.test_all(mask);
    }

    template <typename... T, typename func_t>
    void for_all(func_t callback) {
        if constexpr (sizeof...(T) == 0) {
            for (auto& entity : _entities) if (entity.is_valid) {
                callback(entity.id);
            }
        } else {
            component_mask_t mask{};
            component_id_t component_ids[] = { get_component_id_for<T>()... };
            for (uint32_t i = 0; i < sizeof...(T); i++) {
                mask.set(component_ids[i]);
            }
            for (auto& entity : _entities) if (entity.is_valid && entity.mask.test_all(mask)) {
                callback(entity.id, get<T>(entity.id)...);
            }
        }
    }

private:
    const entity_id_t                       _max_entities;
    std::vector<entity_id_t>                _available_entities;
    std::vector<entity_description_t>       _entities;
    std::vector<base_component_pool_t *>    _component_pools;
};

} // namespace core

#endif