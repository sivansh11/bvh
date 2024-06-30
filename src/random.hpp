#ifndef RANDOM_HPP
#define RANDOM_HPP

#include <random>

namespace core {

class rng_t {
public:
    rng_t(uint32_t seed = 0) : _engine(seed), _distribution_0_to_1(0.f, 1.f) {}

    float randomf() {
        return _distribution_0_to_1(_engine);
    }

    float randomf(float min, float max) {
        return (randomf() * (max - min)) + min;
    }

private:
    std::mt19937 _engine;
    std::uniform_real_distribution<float> _distribution_0_to_1;
};

} // namespace core

#endif