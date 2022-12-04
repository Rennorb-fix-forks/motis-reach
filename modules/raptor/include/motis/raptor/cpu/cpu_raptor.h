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

void invoke_cpu_raptor(schedule const& sched, raptor_query const& query,
                       raptor_meta_info const& reach_values, raptor_statistics&);


void generate_reach_cache(raptor_timetable const& timetable,
                          mcd::vector<station_ptr> const* stations,
                          reach_vals& reach_values);

}  // namespace motis::raptor
