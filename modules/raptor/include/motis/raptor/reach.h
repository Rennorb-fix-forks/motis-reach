#pragma once

#include <vector>
#include <atomic>
#include <motis/core/schedule/constant_graph.h>
#include "motis/raptor/raptor_statistics.h"
#include "motis/routing/label/criteria/travel_time.h"
#include <motis/core/schedule/station.h>

namespace motis::raptor {


using reach_t = float;

struct reach_wrapper : public std::atomic<float> {
  reach_wrapper(reach_t val);
  reach_wrapper(reach_wrapper const& other);
  reach_wrapper& operator=(reach_wrapper const& other);
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
  raptor_statistics& stats_;
};

} // namespace motis::raptor



