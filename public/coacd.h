#pragma once
#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace coacd {

#if defined(_WIN32)
#if defined(COACD_STATIC)
#define COACD_API
#elif defined(_coacd_EXPORTS)
#define COACD_API __declspec(dllexport)
#else
#define COACD_API __declspec(dllimport)
#endif
#else
#define COACD_API
#endif

struct Mesh {
  std::vector<std::array<double, 3>> vertices;
  std::vector<std::array<int, 3>> indices;
};

COACD_API std::vector<Mesh> CoACD(Mesh const &input, double threshold = 0.05,
                        int max_convex_hull = -1, std::string preprocess = "auto",
                        int prep_resolution = 50, int sample_resolution = 2000,
                        int mcts_nodes = 20, int mcts_iteration = 150,
                        int mcts_max_depth = 3, bool pca = false,
                        bool merge = true, bool decimate = false, int max_ch_vertex = 256,
                        bool extrude = false, double extrude_margin = 0.01,
                        std::string apx_mode = "ch", unsigned int seed = 0,
                        bool real_metric = false);
COACD_API void set_log_level(std::string_view level);

} // namespace coacd

extern "C" {

struct CoACD_Mesh {
  double *vertices_ptr;
  uint64_t vertices_count;
  int *triangles_ptr;
  uint64_t triangles_count;
};

struct CoACD_MeshArray {
  CoACD_Mesh *meshes_ptr;
  uint64_t meshes_count;
};

void COACD_API CoACD_freeMeshArray(CoACD_MeshArray arr);

constexpr int preprocess_auto = 0;
constexpr int preprocess_on = 1;
constexpr int preprocess_off = 2;

constexpr int apx_ch = 0;
constexpr int apx_box = 1;

CoACD_MeshArray COACD_API CoACD_run(CoACD_Mesh const &input, double threshold,
                                    int max_convex_hull, int preprocess_mode,
                                    int prep_resolution, int sample_resolution,
                                    int mcts_nodes, int mcts_iteration,
                                    int mcts_max_depth, bool pca, bool merge,
                                    bool decimate, int max_ch_vertex,
                                    bool extrude, double extrude_margin,
                                    int apx_mode, unsigned int seed,
                                    bool real_metric);

/// Never throws: an unknown level is logged and ignored.
void COACD_API CoACD_setLogLevel(char const *level);

// ---------------------------------------------------------------------------
// Fault-tolerant API. Nothing below lets a C++ exception, a bad allocation or
// (on MSVC) a structured exception such as an access violation escape into the
// caller: every failure comes back as a status code plus a message.
// ---------------------------------------------------------------------------

enum CoACD_Status {
  COACD_OK = 0,
  /// A parameter or the input mesh failed validation (nothing was run).
  COACD_ERROR_INVALID_ARGUMENT = 1,
  /// std::bad_alloc: the decomposition ran out of memory.
  COACD_ERROR_BAD_ALLOC = 2,
  /// Any other C++ exception escaped the decomposition.
  COACD_ERROR_EXCEPTION = 3,
  /// A CPU/OS fault (access violation, stack overflow, ...) was intercepted.
  /// The library's internal state may be inconsistent afterwards; treat the
  /// result as an error and prefer restarting the host process before
  /// running another decomposition.
  COACD_ERROR_HARDWARE_FAULT = 4,
  /// The decomposition finished but produced no usable convex piece.
  COACD_ERROR_NO_RESULT = 5,
  /// The decomposition produced non-finite coordinates or bad indices.
  COACD_ERROR_INTERNAL = 6,
  /// `CoACD_Params::time_limit_seconds` passed before the decomposition
  /// finished.
  COACD_ERROR_TIMEOUT = 7,
};

/// Plain-data mirror of the `CoACD_run` arguments (booleans as 0/1).
struct CoACD_Params {
  double threshold;
  int max_convex_hull;
  int preprocess_mode;
  int prep_resolution;
  int sample_resolution;
  int mcts_nodes;
  int mcts_iteration;
  int mcts_max_depth;
  int pca;
  int merge;
  int decimate;
  int max_ch_vertex;
  int extrude;
  double extrude_margin;
  int apx_mode;
  unsigned int seed;
  int real_metric;
  /// Wall-clock budget for one `CoACD_runSafe` call; 0 means no limit. The
  /// decomposition checks it between its search and merge steps (process
  /// wide: one decomposition at a time).
  double time_limit_seconds;
};

/// Fills `params` with the library defaults (threshold 0.05, auto
/// preprocessing at resolution 50, 2000 samples, 20/150/3 MCTS, merge on).
void COACD_API CoACD_defaultParams(CoACD_Params *params);

/// Checks the mesh the way `CoACD_runSafe` does: non-null buffers, at least
/// one vertex and triangle, finite coordinates, in-range indices and a
/// bounding box with a positive, finite extent. Returns a `CoACD_Status` and
/// writes the reason into `message` (NUL-terminated, at most
/// `message_capacity` bytes) when it is not `COACD_OK`.
int COACD_API CoACD_validateMesh(CoACD_Mesh const *input, char *message,
                                 uint64_t message_capacity);

/// Checks the parameters (ranges the algorithm needs to terminate).
int COACD_API CoACD_validateParams(CoACD_Params const *params, char *message,
                                   uint64_t message_capacity);

/// Runs the decomposition. On `COACD_OK`, `*output` holds the pieces and must
/// be released with `CoACD_freeMeshArray`; on any other status `*output` is
/// empty and `message` describes the failure. `message` may be NULL.
int COACD_API CoACD_runSafe(CoACD_Mesh const *input, CoACD_Params const *params,
                            CoACD_MeshArray *output, char *message,
                            uint64_t message_capacity);

/// Static name of a `CoACD_Status` value ("ok", "invalid argument", ...).
COACD_API char const *CoACD_statusName(int status);

/// Version of this build, e.g. "1.0.14-safe".
COACD_API char const *CoACD_version(void);

/// Receives every log line instead of stdout. `level` uses spdlog's
/// numbering (0 trace, 1 debug, 2 info, 3 warn, 4 error, 5 critical). The
/// callback runs on whichever thread logs, including worker threads, and
/// must not throw. NULL restores the stdout sink.
typedef void (*CoACD_LogCallback)(int level, char const *message, void *user);
void COACD_API CoACD_setLogCallback(CoACD_LogCallback callback, void *user);
}
