#ifndef CAMERA_HPP
#define CAMERA_HPP

#include "math/math.hpp"

class camera_t {
 public:
  camera_t(float vfov, math::vec3 from, math::vec3 at,
           math::vec3 up = {0, 1, 0})
      : _vfov(vfov), _from(from), _at(at), _up(up) {}

  void set_dimentions(uint32_t width, uint32_t height) {
    _width  = width;
    _height = height;

    float aspect_ratio = float(_width) / float(_height);

    float focal_length    = length(_from - _at);
    float theta           = math::radians(_vfov);
    float h               = tan(theta / 2.f);
    float viewport_height = 2 * h * focal_length;
    float viewport_width  = viewport_height * (float(_width) / float(_height));

    _w = normalize(_from - _at);
    _u = normalize(cross(_up, _w));
    _v = cross(_w, _u);

    math::vec3 viewport_u = viewport_width * _u;
    math::vec3 viewport_v = viewport_height * -_v;

    _pixel_delta_u = viewport_u / float(_width);
    _pixel_delta_v = viewport_v / float(_height);

    math::vec3 viewport_upper_left =
        _from - (focal_length * _w) - viewport_u / 2.f - viewport_v / 2.f;

    _pixel_00_loc =
        viewport_upper_left + 0.5f * (_pixel_delta_u + _pixel_delta_v);
  }

  std::pair<math::vec3, math::vec3> ray_gen(uint32_t x, uint32_t y) {
    assert(_width != 0 && _height != 0);
    math::vec3 pixel_center = _pixel_00_loc + (float(x) * _pixel_delta_u) +
                              (float(y) * _pixel_delta_v);
    math::vec3 direction = pixel_center - _from;
    return {_from, direction};
  }

  uint32_t _width = 0, _height = 0;

  float      _vfov;
  math::vec3 _from;
  math::vec3 _at;
  math::vec3 _up;

  math::vec3 _pixel_00_loc;
  math::vec3 _pixel_delta_u;
  math::vec3 _pixel_delta_v;
  math::vec3 _u, _v, _w;
};

#endif
