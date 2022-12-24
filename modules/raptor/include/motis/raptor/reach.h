#pragma once

#include <vector>
#include <atomic>

#include "motis/raptor/raptor_statistics.h"
#include "motis/raptor/raptor_query.h"
#include "motis/raptor/raptor_timetable.h"
#include "motis/routing/label/criteria/travel_time.h"
#include "motis/core/schedule/schedule.h"
#include "motis/core/schedule/constant_graph.h"
#include "motis/core/schedule/station.h"

namespace motis::raptor {


using reach_t       = float;
using reach_graph_t = constant_graph_dijkstra<1440, map_station_graph_node>;

struct atomic_reach : public std::atomic<reach_t> {
  atomic_reach(reach_t val) : std::atomic<reach_t>(val) {}
  atomic_reach(atomic_reach const& other) : std::atomic<reach_t>(other.load()) {}
  atomic_reach& operator=(atomic_reach const& other) {
    this->store(other.load());
    return *this;
  }
};

struct raptor_meta_info;
struct raptor_query;

struct ReachEvaluationData {
  schedule      const* sched;
  reach_graph_t        const_graph_travel_time;
  time                 source_time_begin;

  std::vector<reach_t> const* reach_values;

  raptor_statistics * stats;

  ReachEvaluationData(raptor_meta_info const& meta_info, raptor_query const& query, raptor_statistics * stats, schedule const* sched) 
    : sched(sched),
      const_graph_travel_time(sched->travel_time_lower_bounds_fwd_, {query.target_}, {}),
      source_time_begin(query.source_time_begin_),
      reach_values(&meta_info.reach_values_),
      stats(stats)
  {
    const_graph_travel_time.run();
  }
};

inline reach_t dist_metric(ReachEvaluationData const& data, stop_id current_id) {
  // TODO(Rennorb): validate that using index_ is correct here
  return (reach_t)data.const_graph_travel_time[data.sched->station_nodes_[current_id].get()];
}

inline reach_t reach_metric(time src_departure_time, time current_arrival_time) {
  return current_arrival_time - src_departure_time;
}


// returns true if reach values indicate viable path
inline bool test_reach(ReachEvaluationData const& data, stop_id current_id, time current_arrival_time) {
  auto curr_reach = data.reach_values->at(current_id);
  return curr_reach >= reach_metric(data.source_time_begin, current_arrival_time) ||
    curr_reach >= dist_metric(data, current_id);
}

} // namespace motis::raptor



