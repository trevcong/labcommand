// OpenMP shims, so the core also builds (single-threaded) without OpenMP.
#pragma once

#include <cstddef>

#if defined(_OPENMP)
#include <omp.h>
#endif

// NEON kernels (quantizer, ray lanes) on aarch64 unless ROOM_MEMORY_NO_SIMD selects the
// portable lane code, which other architectures always use.
#if defined(__aarch64__) && !defined(ROOM_MEMORY_NO_SIMD)
#define ROOM_MEMORY_NEON 1
#else
#define ROOM_MEMORY_NEON 0
#endif

namespace room_memory::detail {

inline int max_threads() {
#if defined(_OPENMP)
    return omp_get_max_threads();
#else
    return 1;
#endif
}

inline int thread_index() {
#if defined(_OPENMP)
    return omp_get_thread_num();
#else
    return 0;
#endif
}

// Below this many work items a parallel region costs more than it saves.
inline constexpr std::size_t kParallelGrain = 1 << 14;

// Contiguous chunk [begin, end) of `count` items for chunk `part` of `parts`.
inline std::size_t chunk_begin(std::size_t count, std::size_t parts, std::size_t part) {
    return count / parts * part + (part < count % parts ? part : count % parts);
}

}  // namespace room_memory::detail
