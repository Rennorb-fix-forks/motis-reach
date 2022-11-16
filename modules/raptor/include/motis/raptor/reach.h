#pragma once

#include <vector>
#include <atomic>
#include <motis/core/schedule/constant_graph.h>
#include "motis/raptor/raptor_statistics.h"
#include "motis/routing/label/criteria/travel_time.h"

namespace motis::raptor {


using reach_t = float;

struct reach_wrapper : public std::atomic<float> {
  reach_wrapper(reach_t val);
  reach_wrapper(reach_wrapper & other);
};

using reach_vals = std::vector<reach_wrapper>;

struct raptor_meta_info;
struct raptor_query;

struct reach_data {
  // NOTE(Rennorb): Be s,v,t vertices of a graph G and m(x, y, G) a metric.
  //  The reach of a v on a G is the max{ (s,t) : min(m(s,v,G), m(v,t,G)) }
  //  where (x,y) is a least cost path from x to y in G
  reach_vals const& reach_values_;
  time source_time_dep_;
  constant_graph_dijkstra<routing::MAX_TRAVEL_TIME, map_station_graph_node> const_graph_;
  raptor_statistics& stats_;

  reach_data(raptor_meta_info const& meta_info, raptor_query const& query,
             raptor_statistics& stats);
};

} // namespace motis::raptor



