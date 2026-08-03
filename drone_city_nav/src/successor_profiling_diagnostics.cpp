#include "successor_profiling_diagnostics.hpp"

#include <sstream>

namespace drone_city_nav::detail {

std::string
successorProfilingJsonFields(const LatticeSuccessorProfiling& lattice_2d,
                             const Lattice3DSuccessorProfiling& lattice_3d) {
  std::ostringstream output;
  output
      << ",\"successor_search_batches\":" << lattice_2d.search.collection_calls
      << ",\"successor_search_parallel_batches\":"
      << lattice_2d.search.parallel_collection_calls
      << ",\"successor_search_candidates\":" << lattice_2d.search.candidates
      << ",\"successor_search_parallel_candidates\":"
      << lattice_2d.search.parallel_candidates
      << ",\"successor_search_batch_max\":" << lattice_2d.search.maximum_candidates
      << ",\"successor_search_worker_ms\":" << lattice_2d.search.worker_ms
      << ",\"expansion_prefetch_batches\":" << lattice_2d.expansion_prefetch.batches
      << ",\"expansion_prefetch_entries\":" << lattice_2d.expansion_prefetch.entries
      << ",\"expansion_prefetch_parallel_entries\":"
      << lattice_2d.expansion_prefetch.parallel_entries
      << ",\"expansion_prefetch_cache_hits\":"
      << lattice_2d.expansion_prefetch.cache_hits
      << ",\"expansion_prefetch_discarded_entries\":"
      << lattice_2d.expansion_prefetch.discarded_entries
      << ",\"expansion_prefetch_worker_ms\":" << lattice_2d.expansion_prefetch.worker_ms
      << ",\"successor_continuation_batches\":"
      << lattice_2d.continuation.collection_calls
      << ",\"successor_continuation_parallel_batches\":"
      << lattice_2d.continuation.parallel_collection_calls
      << ",\"successor_continuation_candidates\":" << lattice_2d.continuation.candidates
      << ",\"successor_continuation_parallel_candidates\":"
      << lattice_2d.continuation.parallel_candidates
      << ",\"successor_continuation_batch_max\":"
      << lattice_2d.continuation.maximum_candidates
      << ",\"successor_continuation_worker_ms\":" << lattice_2d.continuation.worker_ms
      << ",\"successor_3d_search_batches\":" << lattice_3d.search.collection_calls
      << ",\"successor_3d_search_parallel_batches\":"
      << lattice_3d.search.parallel_collection_calls
      << ",\"successor_3d_search_candidates\":" << lattice_3d.search.candidates
      << ",\"successor_3d_search_parallel_candidates\":"
      << lattice_3d.search.parallel_candidates
      << ",\"successor_3d_search_batch_max\":" << lattice_3d.search.maximum_candidates
      << ",\"successor_3d_search_worker_ms\":" << lattice_3d.search.worker_ms
      << ",\"successor_3d_continuation_batches\":"
      << lattice_3d.continuation.collection_calls
      << ",\"successor_3d_continuation_parallel_batches\":"
      << lattice_3d.continuation.parallel_collection_calls
      << ",\"successor_3d_continuation_candidates\":"
      << lattice_3d.continuation.candidates
      << ",\"successor_3d_continuation_parallel_candidates\":"
      << lattice_3d.continuation.parallel_candidates
      << ",\"successor_3d_continuation_batch_max\":"
      << lattice_3d.continuation.maximum_candidates
      << ",\"successor_3d_continuation_worker_ms\":"
      << lattice_3d.continuation.worker_ms;
  return output.str();
}

} // namespace drone_city_nav::detail
