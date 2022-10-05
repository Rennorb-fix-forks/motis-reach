#pragma once

#include "motis/raptor/cpu/mark_store.h"

#include "motis/raptor/raptor_query.h"
#include "motis/raptor/raptor_result.h"
#include "motis/raptor/raptor_statistics.h"
#include "motis/raptor/raptor_timetable.h"
#include "motis/routing/label/criteria/travel_time.h"

namespace motis::raptor {

trip_count get_earliest_trip(raptor_timetable const& tt,
                             raptor_route const& route,
                             time const* prev_arrivals,
                             stop_times_index r_stop_offset);

void init_arrivals(raptor_result& result, raptor_query const& q,
                   cpu_mark_store& station_marks);

struct reach_data {
  std::vector<float> const& reach_values;
  time source_time_dep;
  constant_graph_dijkstra<routing::MAX_TRAVEL_TIME, map_station_graph_node> const_graph;

  reach_data::reach_data(raptor_meta_info const& meta_info,
                         raptor_query const& query);
};

void update_route(raptor_timetable const& tt, route_id r_id,
                  time const* prev_arrivals, time* current_round,
                  earliest_arrivals& ea, cpu_mark_store& station_marks,
                  reach_data* reach_dat = nullptr);

void update_footpaths(raptor_timetable const& tt, time* current_round,
                      earliest_arrivals const& ea,
                      cpu_mark_store& station_marks);

void invoke_cpu_raptor(schedule const& sched, raptor_query const& query,
                    raptor_meta_info const& reach_values, raptor_statistics&);


void generate_reach_cache(raptor_timetable const& timetable,
                          std::vector<float>& reach_values);

}  // namespace motis::raptor
