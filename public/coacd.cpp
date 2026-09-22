#include "coacd.h"
#include "../src/guard.h"
#include "../src/logger.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <new>
#if WITH_3RD_PARTY_LIBS
#include "../src/preprocess.h"
#endif
#include "../src/process.h"

namespace coacd {
void RecoverParts(vector<Model> &meshes, vector<double> bbox,
                  array<array<double, 3>, 3> rot) {
  for (int i = 0; i < (int)meshes.size(); i++) {
    meshes[i].RevertPCA(rot);
    meshes[i].Recover(bbox);
  }
}

std::vector<Mesh> CoACD(Mesh const &input, double threshold,
                        int max_convex_hull, std::string preprocess_mode,
                        int prep_resolution, int sample_resolution,
                        int mcts_nodes, int mcts_iteration, int mcts_max_depth,
                        bool pca, bool merge, bool decimate, int max_ch_vertex,
                        bool extrude, double extrude_margin,
                        std::string apx_mode, unsigned int seed,
                        bool real_metric) {

  logger::info("threshold               {}", threshold);
  logger::info("max # convex hull       {}", max_convex_hull);
  logger::info("preprocess mode         {}", preprocess_mode);
  logger::info("preprocess resolution   {}", prep_resolution);
  logger::info("pca                     {}", pca);
  logger::info("mcts max depth          {}", mcts_max_depth);
  logger::info("mcts nodes              {}", mcts_nodes);
  logger::info("mcts iterations         {}", mcts_iteration);
  logger::info("merge                   {}", merge);
  logger::info("decimate                {}", decimate);
  logger::info("max_ch_vertex           {}", max_ch_vertex);
  logger::info("extrude                 {}", extrude);
  logger::info("extrude margin          {}", extrude_margin);
  logger::info("approximate mode        {}", apx_mode);
  logger::info("seed                    {}", seed);

  if (!real_metric && threshold > 1) {
    throw std::runtime_error("CoACD threshold > 1 (should be 0-1).");
  }

  if (prep_resolution > 1000) {
    throw std::runtime_error("CoACD prep resolution > 1000, this is probably a "
                             "bug (should be 30-100).");
  } else if (prep_resolution < 5) {
    throw std::runtime_error("CoACD prep resolution < 5, this is probably a "
                             "bug (should be 20-100).");
  }

  Params params;
  params.input_model = "";
  params.output_name = "";
  params.threshold = threshold;
  params.max_convex_hull = max_convex_hull;
  params.preprocess_mode = preprocess_mode;
  params.prep_resolution = prep_resolution;
  params.resolution = sample_resolution;
  params.mcts_nodes = mcts_nodes;
  params.mcts_iteration = mcts_iteration;
  params.mcts_max_depth = mcts_max_depth;
  params.pca = pca;
  params.merge = merge;
  params.decimate = decimate;
  params.max_ch_vertex = max_ch_vertex;
  params.extrude = extrude;
  params.extrude_margin = extrude_margin;
  params.apx_mode = apx_mode;
  params.seed = seed;
  params.real_metric = real_metric;

  Model m;
  m.Load(input.vertices, input.indices);
  vector<double> bbox = m.Normalize();

  if (real_metric) {
    double m_len = max(max(bbox[1] - bbox[0], bbox[3] - bbox[2]), bbox[5] - bbox[4]);
    double original_threshold = params.threshold;
    params.threshold = params.threshold * 2.0 / m_len * 0.8;
    logger::info("Real metric mode: mesh max length = {:.2f} cm", m_len * 100.0);
    logger::info("Real metric mode: error threshold = {:.2f} cm", original_threshold * 100.0);
    logger::info("Real metric mode: threshold {:.4f} cm (real) -> {:.4f} (normalized)", original_threshold * 100.0, params.threshold);
  }


  array<array<double, 3>, 3> rot{
      {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}};

#if WITH_3RD_PARTY_LIBS
  if (params.preprocess_mode == std::string("auto")) {
    bool is_manifold = IsManifold(m);
    logger::info("Mesh Manifoldness: {}", is_manifold);
    if (!is_manifold)
      ManifoldPreprocess(params, m);
  } else if (params.preprocess_mode == std::string("on")) {
    ManifoldPreprocess(params, m);
  }
#else
  bool is_manifold = IsManifold(m);
  if (!is_manifold)
    throw std::runtime_error("The mesh is not a 2-manifold!");
#endif

  if (pca) {
    rot = m.PCA();
  }

  vector<Model> parts = Compute(m, params);
  RecoverParts(parts, bbox, rot);

  std::vector<Mesh> result;
  for (auto &p : parts) {
    result.push_back(Mesh{.vertices = p.points, .indices = p.triangles});
  }
  return result;
}

void set_log_level(std::string_view level) {
#ifndef DISABLE_SPDLOG
  if (level == "off") {
    logger::get()->set_level(spdlog::level::off);
  } else if (level == "debug") {
    logger::get()->set_level(spdlog::level::debug);
  } else if (level == "info") {
    logger::get()->set_level(spdlog::level::info);
  } else if (level == "warn" || level == "warning") {
    logger::get()->set_level(spdlog::level::warn);
  } else if (level == "error" || level == "err") {
    logger::get()->set_level(spdlog::level::err);
  } else if (level == "critical") {
    logger::get()->set_level(spdlog::level::critical);
  } else {
    logger::warn("ignoring unknown log level '{}'", std::string(level));
  }
#endif
}

} // namespace coacd

namespace {

constexpr char const *kVersion = "1.0.14-safe";

void write_message(char *message, uint64_t capacity, std::string const &text) {
  if (!message || capacity == 0) {
    return;
  }
  size_t const n = std::min<size_t>(text.size(), static_cast<size_t>(capacity - 1));
  std::memcpy(message, text.data(), n);
  message[n] = '\0';
}

int fail(int status, std::string const &text, char *message, uint64_t capacity) {
  write_message(message, capacity, text);
  return status;
}

int validate_mesh(CoACD_Mesh const *input, std::string &error) {
  if (!input) {
    error = "mesh pointer is null";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  if (input->vertices_count == 0) {
    error = "mesh has no vertices";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  if (input->triangles_count == 0) {
    error = "mesh has no triangles";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  if (!input->vertices_ptr) {
    error = "vertex buffer is null";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  if (!input->triangles_ptr) {
    error = "triangle buffer is null";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  uint64_t const limit = static_cast<uint64_t>(std::numeric_limits<int>::max()) / 3;
  if (input->vertices_count > limit || input->triangles_count > limit) {
    error = "mesh is too large (" + std::to_string(input->vertices_count) +
            " vertices, " + std::to_string(input->triangles_count) + " triangles)";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  double lo[3] = {std::numeric_limits<double>::infinity(),
                  std::numeric_limits<double>::infinity(),
                  std::numeric_limits<double>::infinity()};
  double hi[3] = {-std::numeric_limits<double>::infinity(),
                  -std::numeric_limits<double>::infinity(),
                  -std::numeric_limits<double>::infinity()};
  for (uint64_t i = 0; i < input->vertices_count; ++i) {
    for (int axis = 0; axis < 3; ++axis) {
      double const value = input->vertices_ptr[3 * i + axis];
      if (!std::isfinite(value)) {
        error = "vertex " + std::to_string(i) + " has a non-finite coordinate";
        return COACD_ERROR_INVALID_ARGUMENT;
      }
      lo[axis] = std::min(lo[axis], value);
      hi[axis] = std::max(hi[axis], value);
    }
  }
  double extent = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    extent = std::max(extent, hi[axis] - lo[axis]);
  }
  if (!std::isfinite(extent)) {
    error = "mesh bounding box overflows a double";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  if (extent <= 0.0) {
    error = "all vertices coincide (the mesh has no extent)";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  uint64_t degenerate = 0;
  for (uint64_t i = 0; i < input->triangles_count; ++i) {
    int const *tri = input->triangles_ptr + 3 * i;
    for (int corner = 0; corner < 3; ++corner) {
      if (tri[corner] < 0 ||
          static_cast<uint64_t>(tri[corner]) >= input->vertices_count) {
        error = "triangle " + std::to_string(i) + " references vertex " +
                std::to_string(tri[corner]) + " but the mesh has " +
                std::to_string(input->vertices_count);
        return COACD_ERROR_INVALID_ARGUMENT;
      }
    }
    if (tri[0] == tri[1] || tri[1] == tri[2] || tri[0] == tri[2]) {
      ++degenerate;
    }
  }
  if (degenerate == input->triangles_count) {
    error = "every triangle is degenerate (repeats a vertex index)";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  if (degenerate > 0) {
    coacd::logger::warn("{} of {} triangles are degenerate", degenerate,
                        input->triangles_count);
  }
  return COACD_OK;
}

int validate_params(CoACD_Params const *params, std::string &error) {
  if (!params) {
    error = "params pointer is null";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  if (!std::isfinite(params->threshold) || params->threshold <= 0.0) {
    error = "threshold must be a positive number";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  if (!params->real_metric && params->threshold > 1.0) {
    error = "threshold > 1 (should be 0.01-1)";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  if (!params->real_metric && params->threshold < 0.001) {
    error = "threshold < 0.001 does not terminate in reasonable time (should be 0.01-1)";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  if (params->max_convex_hull != -1 && params->max_convex_hull < 1) {
    error = "max_convex_hull must be -1 (unlimited) or at least 1";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  if (params->preprocess_mode < preprocess_auto || params->preprocess_mode > preprocess_off) {
    error = "preprocess_mode must be 0 (auto), 1 (on) or 2 (off)";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  if (params->prep_resolution < 5 || params->prep_resolution > 1000) {
    error = "prep_resolution must be 5-1000 (typically 30-100)";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  if (params->sample_resolution < 1) {
    error = "sample_resolution must be at least 1";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  if (params->mcts_nodes < 1 || params->mcts_iteration < 1 || params->mcts_max_depth < 1) {
    error = "mcts_nodes, mcts_iteration and mcts_max_depth must be at least 1";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  if (params->max_ch_vertex < 4) {
    error = "max_ch_vertex must be at least 4";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  if (!std::isfinite(params->extrude_margin) || params->extrude_margin < 0.0) {
    error = "extrude_margin must be a non-negative number";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  if (params->apx_mode != apx_ch && params->apx_mode != apx_box) {
    error = "apx_mode must be 0 (convex hull) or 1 (box)";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  if (!std::isfinite(params->time_limit_seconds) || params->time_limit_seconds < 0.0) {
    error = "time_limit_seconds must be a non-negative number (0 for no limit)";
    return COACD_ERROR_INVALID_ARGUMENT;
  }
  return COACD_OK;
}

/// Copies the pieces into C buffers; frees what it allocated if it fails.
CoACD_MeshArray to_c_array(std::vector<coacd::Mesh> const &meshes) {
  CoACD_MeshArray arr;
  arr.meshes_ptr = new CoACD_Mesh[meshes.size()];
  arr.meshes_count = 0;
  try {
    for (size_t i = 0; i < meshes.size(); ++i) {
      CoACD_Mesh &out = arr.meshes_ptr[i];
      out.vertices_ptr = nullptr;
      out.triangles_ptr = nullptr;
      out.vertices_count = meshes[i].vertices.size();
      out.triangles_count = meshes[i].indices.size();
      arr.meshes_count = i + 1;
      out.vertices_ptr = new double[meshes[i].vertices.size() * 3];
      for (size_t j = 0; j < meshes[i].vertices.size(); ++j) {
        out.vertices_ptr[3 * j] = meshes[i].vertices[j][0];
        out.vertices_ptr[3 * j + 1] = meshes[i].vertices[j][1];
        out.vertices_ptr[3 * j + 2] = meshes[i].vertices[j][2];
      }
      out.triangles_ptr = new int[meshes[i].indices.size() * 3];
      for (size_t j = 0; j < meshes[i].indices.size(); ++j) {
        out.triangles_ptr[3 * j] = meshes[i].indices[j][0];
        out.triangles_ptr[3 * j + 1] = meshes[i].indices[j][1];
        out.triangles_ptr[3 * j + 2] = meshes[i].indices[j][2];
      }
    }
  } catch (...) {
    CoACD_freeMeshArray(arr);
    throw;
  }
  return arr;
}

int run_guarded(CoACD_Mesh const *input, CoACD_Params const *params,
                CoACD_MeshArray *output, std::string &error) {
  coacd::fault_translator_scope translator;
  coacd::deadline_scope deadline(params->time_limit_seconds);
  try {
    coacd::Mesh mesh;
    mesh.vertices.reserve(static_cast<size_t>(input->vertices_count));
    mesh.indices.reserve(static_cast<size_t>(input->triangles_count));
    for (uint64_t i = 0; i < input->vertices_count; ++i) {
      mesh.vertices.push_back({input->vertices_ptr[3 * i],
                               input->vertices_ptr[3 * i + 1],
                               input->vertices_ptr[3 * i + 2]});
    }
    for (uint64_t i = 0; i < input->triangles_count; ++i) {
      mesh.indices.push_back({input->triangles_ptr[3 * i],
                              input->triangles_ptr[3 * i + 1],
                              input->triangles_ptr[3 * i + 2]});
    }
    std::string const pm = params->preprocess_mode == preprocess_on    ? "on"
                           : params->preprocess_mode == preprocess_off ? "off"
                                                                       : "auto";
    std::string const apx = params->apx_mode == apx_box ? "box" : "ch";

    auto meshes = coacd::CoACD(
        mesh, params->threshold, params->max_convex_hull, pm,
        params->prep_resolution, params->sample_resolution, params->mcts_nodes,
        params->mcts_iteration, params->mcts_max_depth, params->pca != 0,
        params->merge != 0, params->decimate != 0, params->max_ch_vertex,
        params->extrude != 0, params->extrude_margin, apx, params->seed,
        params->real_metric != 0);

    if (meshes.empty()) {
      error = "the decomposition produced no convex pieces";
      return COACD_ERROR_NO_RESULT;
    }
    for (size_t i = 0; i < meshes.size(); ++i) {
      for (auto const &v : meshes[i].vertices) {
        if (!std::isfinite(v[0]) || !std::isfinite(v[1]) || !std::isfinite(v[2])) {
          error = "piece " + std::to_string(i) + " has a non-finite coordinate";
          return COACD_ERROR_INTERNAL;
        }
      }
      for (auto const &t : meshes[i].indices) {
        for (int corner = 0; corner < 3; ++corner) {
          if (t[corner] < 0 || static_cast<size_t>(t[corner]) >= meshes[i].vertices.size()) {
            error = "piece " + std::to_string(i) + " has an out-of-range triangle index";
            return COACD_ERROR_INTERNAL;
          }
        }
      }
    }
    *output = to_c_array(meshes);
    return COACD_OK;
  } catch (coacd::hardware_fault const &fault) {
    coacd::after_fault(fault.code);
    error = std::string("hardware fault inside CoACD: ") + fault.what();
    return COACD_ERROR_HARDWARE_FAULT;
  } catch (coacd::timeout_error const &timeout) {
    error = timeout.what();
    return COACD_ERROR_TIMEOUT;
  } catch (std::bad_alloc const &) {
    error = "out of memory";
    return COACD_ERROR_BAD_ALLOC;
  } catch (std::exception const &exception) {
    error = exception.what();
    return COACD_ERROR_EXCEPTION;
  } catch (...) {
    error = "unknown C++ exception";
    return COACD_ERROR_EXCEPTION;
  }
}

} // namespace

extern "C" {
void CoACD_freeMeshArray(CoACD_MeshArray arr) {
  if (!arr.meshes_ptr) {
    return;
  }
  for (uint64_t i = 0; i < arr.meshes_count; ++i) {
    delete[] arr.meshes_ptr[i].vertices_ptr;
    arr.meshes_ptr[i].vertices_ptr = nullptr;
    arr.meshes_ptr[i].vertices_count = 0;
    delete[] arr.meshes_ptr[i].triangles_ptr;
    arr.meshes_ptr[i].triangles_ptr = nullptr;
    arr.meshes_ptr[i].triangles_count = 0;
  }
  delete[] arr.meshes_ptr;
}

void CoACD_defaultParams(CoACD_Params *params) {
  if (!params) {
    return;
  }
  params->threshold = 0.05;
  params->max_convex_hull = -1;
  params->preprocess_mode = preprocess_auto;
  params->prep_resolution = 50;
  params->sample_resolution = 2000;
  params->mcts_nodes = 20;
  params->mcts_iteration = 150;
  params->mcts_max_depth = 3;
  params->pca = 0;
  params->merge = 1;
  params->decimate = 0;
  params->max_ch_vertex = 256;
  params->extrude = 0;
  params->extrude_margin = 0.01;
  params->apx_mode = apx_ch;
  params->seed = 0;
  params->real_metric = 0;
  params->time_limit_seconds = 0.0;
}

int CoACD_validateMesh(CoACD_Mesh const *input, char *message,
                       uint64_t message_capacity) {
  try {
    std::string error;
    int const status = validate_mesh(input, error);
    if (status != COACD_OK) {
      write_message(message, message_capacity, error);
    }
    return status;
  } catch (std::exception const &exception) {
    return fail(COACD_ERROR_EXCEPTION, exception.what(), message, message_capacity);
  } catch (...) {
    return fail(COACD_ERROR_EXCEPTION, "unknown C++ exception", message, message_capacity);
  }
}

int CoACD_validateParams(CoACD_Params const *params, char *message,
                         uint64_t message_capacity) {
  try {
    std::string error;
    int const status = validate_params(params, error);
    if (status != COACD_OK) {
      write_message(message, message_capacity, error);
    }
    return status;
  } catch (std::exception const &exception) {
    return fail(COACD_ERROR_EXCEPTION, exception.what(), message, message_capacity);
  } catch (...) {
    return fail(COACD_ERROR_EXCEPTION, "unknown C++ exception", message, message_capacity);
  }
}

int CoACD_runSafe(CoACD_Mesh const *input, CoACD_Params const *params,
                  CoACD_MeshArray *output, char *message,
                  uint64_t message_capacity) {
  if (!output) {
    return fail(COACD_ERROR_INVALID_ARGUMENT, "output pointer is null", message,
                message_capacity);
  }
  output->meshes_ptr = nullptr;
  output->meshes_count = 0;
  try {
    std::string error;
    int status = validate_params(params, error);
    if (status == COACD_OK) {
      status = validate_mesh(input, error);
    }
    if (status == COACD_OK) {
      status = run_guarded(input, params, output, error);
    }
    if (status != COACD_OK) {
      write_message(message, message_capacity, error);
    }
    return status;
  } catch (std::bad_alloc const &) {
    return fail(COACD_ERROR_BAD_ALLOC, "out of memory", message, message_capacity);
  } catch (std::exception const &exception) {
    return fail(COACD_ERROR_EXCEPTION, exception.what(), message, message_capacity);
  } catch (...) {
    return fail(COACD_ERROR_EXCEPTION, "unknown C++ exception", message, message_capacity);
  }
}

char const *CoACD_statusName(int status) {
  switch (status) {
  case COACD_OK:
    return "ok";
  case COACD_ERROR_INVALID_ARGUMENT:
    return "invalid argument";
  case COACD_ERROR_BAD_ALLOC:
    return "out of memory";
  case COACD_ERROR_EXCEPTION:
    return "exception";
  case COACD_ERROR_HARDWARE_FAULT:
    return "hardware fault";
  case COACD_ERROR_NO_RESULT:
    return "no result";
  case COACD_ERROR_INTERNAL:
    return "internal error";
  case COACD_ERROR_TIMEOUT:
    return "time limit exceeded";
  default:
    return "unknown status";
  }
}

char const *CoACD_version(void) { return kVersion; }

void CoACD_setLogCallback(CoACD_LogCallback callback, void *user) {
  try {
    coacd::logger::set_callback(callback, user);
  } catch (...) {
  }
}

/// Legacy entry point: same contract as before on success; every failure is
/// logged and reported as an empty array instead of terminating the caller.
CoACD_MeshArray CoACD_run(CoACD_Mesh const &input, double threshold,
                          int max_convex_hull, int preprocess_mode,
                          int prep_resolution, int sample_resolution,
                          int mcts_nodes, int mcts_iteration,
                          int mcts_max_depth, bool pca, bool merge,
                          bool decimate, int max_ch_vertex,
                          bool extrude, double extrude_margin,
                          int apx_mode, unsigned int seed,
                          bool real_metric) {
  CoACD_Params params;
  params.threshold = threshold;
  params.max_convex_hull = max_convex_hull;
  params.preprocess_mode = preprocess_mode;
  params.prep_resolution = prep_resolution;
  params.sample_resolution = sample_resolution;
  params.mcts_nodes = mcts_nodes;
  params.mcts_iteration = mcts_iteration;
  params.mcts_max_depth = mcts_max_depth;
  params.pca = pca ? 1 : 0;
  params.merge = merge ? 1 : 0;
  params.decimate = decimate ? 1 : 0;
  params.max_ch_vertex = max_ch_vertex;
  params.extrude = extrude ? 1 : 0;
  params.extrude_margin = extrude_margin;
  params.apx_mode = apx_mode;
  params.seed = seed;
  params.real_metric = real_metric ? 1 : 0;
  params.time_limit_seconds = 0.0;

  CoACD_MeshArray arr;
  char message[512];
  int const status = CoACD_runSafe(&input, &params, &arr, message, sizeof message);
  if (status != COACD_OK) {
    try {
      coacd::logger::error("CoACD_run failed ({}): {}", CoACD_statusName(status), message);
    } catch (...) {
    }
    arr.meshes_ptr = nullptr;
    arr.meshes_count = 0;
  }
  return arr;
}

void CoACD_setLogLevel(char const *level) {
  try {
    coacd::set_log_level(std::string_view(level ? level : ""));
  } catch (...) {
  }
}
}
