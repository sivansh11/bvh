#include <getopt.h>

#include <cassert>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "bvh/bvh.hpp"
#include "bvh/traversal.hpp"
#include "glm/common.hpp"
#include "math/triangle.hpp"
#include "math/utilies.hpp"
#include "model/model.hpp"

float clamp(float val, float min, float max) {
  return val > max ? max : val < min ? min : val;
}

struct image_t {
  image_t(uint32_t width, uint32_t height) : width(width), height(height) {
    _p_pixels = new math::vec4[width * height];
  }
  ~image_t() { delete[] _p_pixels; }

  math::vec4 &at(uint32_t x, uint32_t y) {
    assert(y * width + x < width * height);
    return _p_pixels[y * width + x];
  }

  void to_disk(const std::filesystem::path &path) {
    std::stringstream s;
    s << "P3\n" << width << ' ' << height << "\n255\n";
    for (int j = height - 1; j >= 0; j--)
      for (int i = 0; i < width; i++) {
        math::vec4 pixel = at(i, j);
        s << uint32_t(clamp(pixel.r, 0, 1) * 255) << ' '
          << uint32_t(clamp(pixel.g, 0, 1) * 255) << ' '
          << uint32_t(clamp(pixel.b, 0, 1) * 255) << '\n';
      }

    std::ofstream file{path};
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open file");
    }
    file << s.str();
    file.close();
  }

  uint32_t    width, height;
  math::vec4 *_p_pixels;
};

class camera_t {
 public:
  camera_t(float vfov, math::vec3 from, math::vec3 at,
           math::vec3 up = {0, 1, 0})
      : _vfov(vfov), _from(from), _at(at), _up(up) {}

  void set_dimensions(uint32_t width, uint32_t height) {
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
    math::vec3 direction    = pixel_center - _from;
    return {_from, direction};
  }

 private:
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

math::vec3 random_color_from_hit(uint32_t v) {
  return {(((v + 1) * 123) % 255) / 255.f, (((v + 1) * 456) % 255) / 255.f,
          (((v + 1) * 789) % 255) / 255.f};
}

math::vec4 turbo_color_map(float x) {
  // Source:
  // https://research.google/blog/turbo-an-improved-rainbow-colormap-for-visualization/

  const math::vec4 kRedVec4 =
      math::vec4(0.13572138, 4.61539260, -42.66032258, 132.13108234);
  const math::vec4 kGreenVec4 =
      math::vec4(0.09140261, 2.19418839, 4.84296658, -14.18503333);
  const math::vec4 kBlueVec4 =
      math::vec4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
  const math::vec2 kRedVec2   = math::vec2(-152.94239396, 59.28637943);
  const math::vec2 kGreenVec2 = math::vec2(4.27729857, 2.82956604);
  const math::vec2 kBlueVec2  = math::vec2(-89.90310912, 27.34824973);

  x             = clamp(x, 0, 1);
  math::vec4 v4 = math::vec4(1.0, x, x * x, x * x * x);
  math::vec2 v2 = math::vec2{v4.z, v4.w} * v4.z;
  return math::vec4(dot(v4, kRedVec4) + dot(v2, kRedVec2),
                    dot(v4, kGreenVec4) + dot(v2, kGreenVec2),
                    dot(v4, kBlueVec4) + dot(v2, kBlueVec2), 1);
}

enum class builder_type_t : uint32_t {
  e_binned_sah,
  e_sweep_sah,
  e_ploc,
};

enum class pre_processing_type_t : uint32_t {
  e_none     = 1 << 0,
  e_presplit = 1 << 1,
};

enum class post_processing_type_t : uint32_t {
  e_none                     = 1 << 0,
  e_reinsertion_optimisation = 1 << 1,
};

struct binned_sah_builder_opts_t {
  uint32_t num_samples    = 8;
  uint32_t min_primitives = 1;
  uint32_t max_primitives = 1;
};
struct ploc_builder_opts_t {
  uint32_t grid_dim      = 1024;
  uint32_t log_bits      = 10;
  uint32_t search_radius = 15;
};
struct sweep_sah_builder_opts_t {
  uint32_t min_primitives = 1;
  uint32_t max_primitives = 1;
};
struct builder_opts_t {
  builder_opts_t()
      : type(builder_type_t::e_binned_sah),
        binned_sah(binned_sah_builder_opts_t{}) {}
  builder_type_t type;
  union {
    binned_sah_builder_opts_t binned_sah;
    sweep_sah_builder_opts_t  sweep_sah;
    ploc_builder_opts_t       ploc;
  };
};

struct presplit_opts_t {
  float split_factor = 0.3f;
};

struct reinsert_opts_t {
  float    batch_size_ratio = 0.1f;
  uint32_t max_itr          = 10;
};

struct options_t {
  std::filesystem::path  model_path;
  uint32_t               reruns  = 10;
  uint32_t               threads = 8;
  builder_opts_t         builder;
  pre_processing_type_t  pre_processing = pre_processing_type_t::e_none;
  presplit_opts_t        presplit;
  post_processing_type_t post_processing = post_processing_type_t::e_none;
  reinsert_opts_t        reinsert;
  bool                   use_soa = false;
};

void print_usage(std::ostream &stream) {
  stream
      << "Usage: benchmark [options] -m <model>\n"
      << "\n"
      << "Options:\n"
      << "  -m, --model <path>          model file path (required)\n"
      << "  -n, --reruns <count>        number of render reruns (default 10)\n"
      << "  -t, --threads <count>      number of render threads (default 8)\n"
      << "  -b, --builder <builder>     builder algorithm and per-builder "
         "options\n"
      << "                                "
         "binned_sah[=num_samples=X,min_primitives=X,max_primitives=X]\n"
      << "                                "
         "ploc[=grid_dim=X,log_bits=X,search_radius=X]\n"
      << "                                "
         "sweep_sah[=min_primitives=X,max_primitives=X]\n"
      << "      --presplit             enable presplit pre-processing step\n"
      << "      --presplit-factor <f>   presplit split factor (default 0.3)\n"
      << "      --reinsert             enable reinsertion optimization "
         "post-processing\n"
      << "      --reinsert-ratio <f>   reinsertion batch size ratio (default "
         "0.1)\n"
      << "      --reinsert-itr <n>     reinsertion max iterations (default "
         "10)\n"
      << "      --soa                 use SoA (vec4-padded) BVH traversal "
         "instead of AoS\n"
      << "  -h, --help                  show this help and exit\n";
}

uint32_t parse_u32(const std::string &key, const std::string &value) {
  try {
    size_t   pos = 0;
    uint32_t v   = std::stoul(value, &pos);
    if (pos != value.size()) throw std::invalid_argument("non-numeric");
    return v;
  } catch (const std::exception &) {
    throw std::runtime_error("Invalid value '" + value + "' for option '" +
                             key + "'");
  }
}

float parse_float(const std::string &key, const std::string &value) {
  try {
    size_t pos = 0;
    float  v   = std::stof(value, &pos);
    if (pos != value.size()) throw std::invalid_argument("non-numeric");
    return v;
  } catch (const std::exception &) {
    throw std::runtime_error("Invalid value '" + value + "' for option '" +
                             key + "'");
  }
}

void apply_option_value(binned_sah_builder_opts_t &opts, const std::string &key,
                        const std::string &value) {
  if (key == "num_samples")
    opts.num_samples = parse_u32(key, value);
  else if (key == "min_primitives")
    opts.min_primitives = parse_u32(key, value);
  else if (key == "max_primitives")
    opts.max_primitives = parse_u32(key, value);
  else
    throw std::runtime_error("Unknown option '" + key +
                             "' for binned_sah builder");
}

void apply_option_value(ploc_builder_opts_t &opts, const std::string &key,
                        const std::string &value) {
  if (key == "grid_dim")
    opts.grid_dim = parse_u32(key, value);
  else if (key == "log_bits")
    opts.log_bits = parse_u32(key, value);
  else if (key == "search_radius")
    opts.search_radius = parse_u32(key, value);
  else
    throw std::runtime_error("Unknown option '" + key + "' for ploc builder");
}

void apply_option_value(sweep_sah_builder_opts_t &opts, const std::string &key,
                        const std::string &value) {
  if (key == "min_primitives")
    opts.min_primitives = parse_u32(key, value);
  else if (key == "max_primitives")
    opts.max_primitives = parse_u32(key, value);
  else
    throw std::runtime_error("Unknown option '" + key +
                             "' for sweep_sah builder");
}

template <typename T>
void apply_option_value(T &, const std::string &key, const std::string &) {
  throw std::runtime_error("Unknown option '" + key + "'");
}

void parse_builder_sub_options(builder_opts_t    &builder,
                               const std::string &sub) {
  if (sub.empty()) return;
  size_t start = 0;
  while (start <= sub.size()) {
    size_t      comma = sub.find(',', start);
    std::string pair  = sub.substr(
        start, comma == std::string::npos ? sub.size() - start : comma - start);
    size_t eq = pair.find('=');
    if (eq == std::string::npos)
      throw std::runtime_error("Expected key=value in '" + pair + "'");
    std::string key   = pair.substr(0, eq);
    std::string value = pair.substr(eq + 1);

    switch (builder.type) {
      case builder_type_t::e_binned_sah:
        apply_option_value(builder.binned_sah, key, value);
        break;
      case builder_type_t::e_sweep_sah:
        apply_option_value(builder.sweep_sah, key, value);
        break;
      case builder_type_t::e_ploc:
        apply_option_value(builder.ploc, key, value);
        break;
    }

    if (comma == std::string::npos) break;
    start = comma + 1;
  }
}

builder_opts_t parse_builder(const std::string &arg) {
  std::string name  = arg, sub;
  size_t      colon = arg.find(':');
  if (colon != std::string::npos) {
    name = arg.substr(0, colon);
    sub  = arg.substr(colon + 1);
  }

  builder_opts_t builder;
  if (name == "binned_sah") {
    builder.type       = builder_type_t::e_binned_sah;
    builder.binned_sah = binned_sah_builder_opts_t{};
  } else if (name == "sweep_sah") {
    builder.type      = builder_type_t::e_sweep_sah;
    builder.sweep_sah = sweep_sah_builder_opts_t{};
  } else if (name == "ploc") {
    builder.type = builder_type_t::e_ploc;
    builder.ploc = ploc_builder_opts_t{};
  } else
    throw std::runtime_error("Unknown builder '" + name +
                             "' (expected binned_sah, ploc, or sweep_sah)");

  parse_builder_sub_options(builder, sub);
  return builder;
}

bvh::bvh_t build_with(const builder_opts_t            &builder,
                      const std::vector<math::aabb_t> &aabbs) {
  switch (builder.type) {
    case builder_type_t::e_binned_sah: {
      const auto &opts = builder.binned_sah;
      return bvh::build_bvh_binned_sah(
          aabbs, opts.num_samples, opts.min_primitives, opts.max_primitives);
    }
    case builder_type_t::e_sweep_sah: {
      const auto &opts = builder.sweep_sah;
      return bvh::build_bvh_sweep_sah(aabbs, opts.min_primitives,
                                      opts.max_primitives);
    }
    case builder_type_t::e_ploc: {
      const auto &opts = builder.ploc;
      return bvh::build_bvh_ploc(aabbs, opts.grid_dim, opts.log_bits,
                                 opts.search_radius);
    }
  }
  assert(false);
  return {};
}

options_t parse_options(int argc, char **argv) {
  options_t opts;

  static const struct option long_options[] = {
      {"model", required_argument, nullptr, 'm'},
      {"reruns", required_argument, nullptr, 'n'},
      {"threads", required_argument, nullptr, 't'},
      {"builder", required_argument, nullptr, 'b'},
      {"presplit", no_argument, nullptr, 1000},
      {"presplit-factor", required_argument, nullptr, 1001},
      {"reinsert", no_argument, nullptr, 1002},
      {"reinsert-ratio", required_argument, nullptr, 1003},
      {"reinsert-itr", required_argument, nullptr, 1004},
      {"soa", no_argument, nullptr, 1005},
      {"help", no_argument, nullptr, 'h'},
      {nullptr, 0, nullptr, 0}};

  int opt;
  while ((opt = getopt_long(argc, argv, "m:n:t:b:h", long_options, nullptr)) !=
         -1) {
    switch (opt) {
      case 'm':
        opts.model_path = optarg;
        break;
      case 'n':
        opts.reruns = parse_u32("reruns", optarg);
        break;
      case 't':
        opts.threads = parse_u32("threads", optarg);
        break;
      case 'b':
        opts.builder = parse_builder(optarg);
        break;
      case 1000:
        opts.pre_processing = static_cast<pre_processing_type_t>(
            static_cast<uint32_t>(opts.pre_processing) |
            static_cast<uint32_t>(pre_processing_type_t::e_presplit));
        break;
      case 1001:
        opts.presplit.split_factor = parse_float("presplit-factor", optarg);
        break;
      case 1002:
        opts.post_processing = static_cast<post_processing_type_t>(
            static_cast<uint32_t>(opts.post_processing) |
            static_cast<uint32_t>(
                post_processing_type_t::e_reinsertion_optimisation));
        break;
      case 1003:
        opts.reinsert.batch_size_ratio = parse_float("reinsert-ratio", optarg);
        break;
      case 1004:
        opts.reinsert.max_itr = parse_u32("reinsert-itr", optarg);
        break;
      case 1005:
        opts.use_soa = true;
        break;
      case 'h':
      default:
        print_usage(std::cout);
        exit(EXIT_FAILURE);
    }
  }

  if (optind < argc)
    throw std::runtime_error("Unexpected positional argument '" +
                             std::string(argv[optind]) + "'");
  if (opts.model_path.empty())
    throw std::runtime_error("Missing required option '-m <model>'");

  return opts;
}

template <typename fn_t>
void render(uint32_t threads, image_t &image, fn_t fn) {
  std::vector<std::pair<uint32_t, uint32_t>> work;
  for (uint32_t y = 0; y < image.height; y++)
    for (uint32_t x = 0; x < image.width; x++) work.emplace_back(x, y);

  if (threads <= 1) {
    for (const auto &[x, y] : work)
      image.at(x, image.height - y - 1) = fn(x, y);
    return;
  }

  std::vector<std::thread> workers;
  for (uint32_t thread_index = 0; thread_index < threads; thread_index++) {
    workers.emplace_back(
        [&](uint32_t thread_index) {
          for (uint32_t work_index = thread_index; work_index < work.size();
               work_index += threads) {
            auto [x, y]                       = work[work_index];
            image.at(x, image.height - y - 1) = fn(x, y);
          }
        },
        thread_index);
  }
  for (auto &worker : workers) worker.join();
}

void print_config(const options_t &opts, std::ostream &stream) {
  const char *builder_name = nullptr;
  switch (opts.builder.type) {
    case builder_type_t::e_binned_sah:
      builder_name = "binned_sah";
      break;
    case builder_type_t::e_sweep_sah:
      builder_name = "sweep_sah";
      break;
    case builder_type_t::e_ploc:
      builder_name = "ploc";
      break;
  }

  stream << "builder: " << builder_name;
  switch (opts.builder.type) {
    case builder_type_t::e_binned_sah: {
      const auto &o = opts.builder.binned_sah;
      stream << " (num_samples=" << o.num_samples
             << ", min_primitives=" << o.min_primitives
             << ", max_primitives=" << o.max_primitives << ")";
      break;
    }
    case builder_type_t::e_sweep_sah: {
      const auto &o = opts.builder.sweep_sah;
      stream << " (min_primitives=" << o.min_primitives
             << ", max_primitives=" << o.max_primitives << ")";
      break;
    }
    case builder_type_t::e_ploc: {
      const auto &o = opts.builder.ploc;
      stream << " (grid_dim=" << o.grid_dim << ", log_bits=" << o.log_bits
             << ", search_radius=" << o.search_radius << ")";
      break;
    }
  }
  stream << '\n';

  if ((static_cast<uint32_t>(opts.pre_processing) &
       static_cast<uint32_t>(pre_processing_type_t::e_presplit)) != 0) {
    stream << "pre-processing: presplit (split_factor="
           << opts.presplit.split_factor << ")\n";
  }

  if ((static_cast<uint32_t>(opts.post_processing) &
       static_cast<uint32_t>(
           post_processing_type_t::e_reinsertion_optimisation)) != 0) {
    stream << "post-processing: reinsertion_optimisation "
              "(batch_size_ratio="
           << opts.reinsert.batch_size_ratio
           << ", max_itr=" << opts.reinsert.max_itr << ")\n";
  }

  stream << "reruns: " << opts.reruns << " threads: " << opts.threads
         << " traversal: " << (opts.use_soa ? "soa" : "aos") << '\n';
}

int main(int argc, char **argv) {
  options_t opts;
  try {
    opts = parse_options(argc, argv);
  } catch (const std::exception &e) {
    std::cerr << "Error: " << e.what() << "\n\n";
    print_usage(std::cerr);
    exit(EXIT_FAILURE);
  }

  print_config(opts, std::cout);

  model::raw_model_t model = model::load_model_from_path(opts.model_path);
  model                    = model::merge_meshes(model);
  model::raw_mesh_t &mesh  = model.meshes[0];

  auto start     = std::chrono::high_resolution_clock::now();
  auto triangles = model::create_triangles_from_mesh(mesh);

  bvh::bvh_t bvh;
  bool       is_presplit =
      (static_cast<uint32_t>(opts.pre_processing) &
       static_cast<uint32_t>(pre_processing_type_t::e_presplit)) != 0;
  if (is_presplit) {
    auto [aabbs, tri_indices] =
        bvh::presplit(triangles, opts.presplit.split_factor);
    bvh = build_with(opts.builder, aabbs);
    bvh::presplit_remove_indirection(bvh, tri_indices);
    bvh::presplit_remove_duplicates(bvh);
  } else {
    auto aabbs = math::aabbs_from_triangles(triangles);
    bvh        = build_with(opts.builder, aabbs);
  }
  bool is_reinsert =
      (static_cast<uint32_t>(opts.post_processing) &
       static_cast<uint32_t>(
           post_processing_type_t::e_reinsertion_optimisation)) != 0;
  if (is_reinsert)
    bvh::reinsertion_optimize(bvh, opts.reinsert.batch_size_ratio,
                              opts.reinsert.max_itr);
  auto end = std::chrono::high_resolution_clock::now();
  std::cout << "builder took: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                     start)
                   .count()
            << "ms\n";

  std::cout << "depth of bvh: " << bvh::depth_of_bvh(bvh) << '\n';
  float sah = bvh::sah_of_bvh(bvh);
  float epo = bvh::epo_of_bvh(bvh, triangles, opts.threads);
  std::cout << "sah of bvh: " << sah << '\n';
  std::cout << "epo of bvh: " << epo << '\n';
  float alpha = 0.71;
  std::cout << "sah-epo: " << ((1.f - alpha) * sah + alpha * epo) << '\n';
  std::cout << "num nodes: " << bvh.nodes.size() << '\n';

  image_t image{640, 420};
  // camera_t camera{90.f, {0, 1, 2}, {0, 1, 0}};
  camera_t camera{90.f, {0, 1, 0}, {-1, 1, 0}};
  // camera_t camera{90.f, {0, 50, 0}, {-1, 50, 0}};
  camera.set_dimensions(image.width, image.height);

  bvh::soa_bvh_t soa_bvh;
  if (opts.use_soa) soa_bvh = bvh::convert(bvh);

  for (uint32_t i = 0; i < opts.reruns; i++) {
    start = std::chrono::high_resolution_clock::now();
    render(opts.threads, image, [&](uint32_t x, uint32_t y) {
      auto [O, D]    = camera.ray_gen(x, y);
      bvh::ray_t ray = bvh::ray_t::create(O, D);
      bvh::hit_t hit;
      if (opts.use_soa) {
        hit = bvh::intersect_soa_bvh(soa_bvh, triangles.data(), ray);
      } else {
        hit = bvh::intersect_bvh(bvh.nodes.data(), bvh.prim_indices.data(),
                                 triangles.data(), ray);
      }
      // if (hit.did_intersect()) {
      return turbo_color_map(
          (hit.node_intersections + hit.triangle_intersections * 1.1f) / 150.f);
      // } else {
      // return math::vec4{0, 0, 0, 0};
      // }
    });
    end = std::chrono::high_resolution_clock::now();
    std::cout << "render took: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(end -
                                                                       start)
                     .count()
              << "ms" << "; "  //
              << "average time taken per ray: "
              << float(std::chrono::duration_cast<std::chrono::nanoseconds>(
                           end - start)
                           .count()) /
                     float(image.width * image.height)
              << "ns"
              << "\n";
  }

  image.to_disk("test.ppm");

  return 0;
}
