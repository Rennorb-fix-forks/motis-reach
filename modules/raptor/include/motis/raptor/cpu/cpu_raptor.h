#pragma once

#include "motis/raptor/cpu/mark_store.h"
#include "motis/raptor/raptor_query.h"
#include "motis/raptor/raptor_result.h"
#include "motis/raptor/raptor_statistics.h"
#include "motis/raptor/raptor_timetable.h"
#include "motis/raptor/reach.h"
#include "motis/core/schedule/schedule.h"

namespace motis::raptor {

trip_count get_earliest_trip(raptor_timetable const& tt,
                             raptor_route const& route,
                             time const* prev_arrivals,
                             stop_times_index r_stop_offset);

void invoke_cpu_raptor(schedule const& sched, raptor_query const& query,
                       raptor_meta_info const& reach_values, raptor_statistics&);

void update_route(raptor_timetable const& tt, route_id const r_id,
  time const* const prev_arrivals, time* const current_round,
  earliest_arrivals& ea, cpu_mark_store& station_marks,
  std::optional<ReachEvaluationData> const reach_dat = std::nullopt);

void update_footpaths(raptor_timetable const& tt, time* current_round,
  earliest_arrivals const& ea, cpu_mark_store& station_marks,
  std::optional<ReachEvaluationData> const reach_dat = std::nullopt);

void generate_reach_values(ReachGenerationData & request);

}  // namespace motis::raptor
