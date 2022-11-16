#pragma once

#include "motis/raptor/cpu/mark_store.h"

#include "motis/raptor/raptor_query.h"
#include "motis/raptor/raptor_result.h"
#include "motis/raptor/raptor_statistics.h"
#include "motis/raptor/raptor_timetable.h"
#include "motis/raptor/reach.h"

namespace motis::raptor {

trip_count get_earliest_trip(raptor_timetable const& tt,
                             raptor_route const& route,
                             time const* prev_arrivals,
                             stop_times_index r_stop_offset);

void init_arrivals(raptor_result& result, raptor_query const& q,
                   cpu_mark_store& station_marks);


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
                          reach_vals& reach_values);

}  // namespace motis::raptor
