#include <unordered_set>

#include "motis/raptor/raptor.h"
#include "motis/raptor/cpu/cpu_raptor.h"
#include "motis/core/common/logging.h"
#include "motis/core/common/timing.h"
#include "motis/routing/label/criteria/travel_time.h"
#include "utl/progress_tracker.h"

namespace motis::raptor {

trip_count get_earliest_trip(raptor_timetable const& tt,
                             raptor_route const& route,
                             time const* const prev_arrivals,
                             stop_times_index const r_stop_offset) {

  stop_id const stop_id =
      tt.route_stops_[route.index_to_route_stops_ + r_stop_offset];

  // station was never visited, there can't be a earliest trip
  if (!valid(prev_arrivals[stop_id])) {
    return invalid<trip_count>;
  }

  // get first defined earliest trip for the stop in the route
  auto const first_trip_stop_idx = route.index_to_stop_times_ + r_stop_offset;
  auto const last_trip_stop_idx =
      first_trip_stop_idx + ((route.trip_count_ - 1) * route.stop_count_);

  trip_count current_trip = 0;
  for (auto stop_time_idx = first_trip_stop_idx;
       stop_time_idx <= last_trip_stop_idx;
       stop_time_idx += route.stop_count_) {

    auto const stop_time = tt.stop_times_[stop_time_idx];
    if (valid(stop_time.departure_) &&
        prev_arrivals[stop_id] <= stop_time.departure_) {
      return current_trip;
    }

    ++current_trip;
  }

  return invalid<trip_count>;
}

void init_arrivals(raptor_result& result, raptor_query const& q,
                   cpu_mark_store& station_marks) {

  // Don't set the values for the earliest arrival, as the footpath update
  // in the first round will use the values in conjunction with the
  // footpath lengths without transfertime leading to invalid results.
  // Not setting the earliest arrival values should (I hope) be correct.
  result[0][q.source_] = q.source_time_begin_;
  station_marks.mark(q.source_);

  for (auto const& add_start : q.add_starts_) {
    time const add_start_time = q.source_time_begin_ + add_start.offset_;
    result[0][add_start.s_id_] =
        std::min(result[0][add_start.s_id_], add_start_time);
    station_marks.mark(add_start.s_id_);
  }
}

// these could be modified
float dist_metric(time src_time, time dst_time) {
  return static_cast<float>(dst_time - src_time);
}
float reach_metric(time src_time, time dst_time) {
  return dist_metric(src_time, dst_time);
}

// returns true if reach values indicate viable path
bool test_reach(std::vector<float> const& reach_values, time src_time,
                stop_id curr_stop, time curr_stop_time, time dst_time) {
  return reach_values[curr_stop] >= reach_metric(src_time, curr_stop_time)
         || reach_values[curr_stop] >= dist_metric(curr_stop_time, dst_time);
}

void update_route(raptor_timetable const& tt, route_id const r_id,
                  time const* const prev_arrivals, time* const current_round,
                  earliest_arrivals& ea, cpu_mark_store& station_marks,
                  reach_data* reach_dat) {
  auto const& route = tt.routes_[r_id];

  trip_count earliest_trip_id = invalid<trip_count>;
  for (stop_id r_stop_offset = 0; r_stop_offset < route.stop_count_;
       ++r_stop_offset) {

    if (!valid(earliest_trip_id)) {
      earliest_trip_id =
          get_earliest_trip(tt, route, prev_arrivals, r_stop_offset);
      continue;
    }

    auto const stop_id =
        tt.route_stops_[route.index_to_route_stops_ + r_stop_offset];
    auto const current_stop_time_idx = route.index_to_stop_times_ +
                                       (earliest_trip_id * route.stop_count_) +
                                       r_stop_offset;

    auto const& stop_time = tt.stop_times_[current_stop_time_idx];

    // need the minimum due to footpaths updating arrivals
    // and not earliest arrivals
    auto const min = std::min(current_round[stop_id], ea[stop_id]);

    if (reach_dat) {
      reach_dat->stats.raptor_station_queries_++;
      if (!test_reach(reach_dat->reach_values, reach_dat->source_time_dep,
                      stop_id, min, reach_dat->const_graph.dists_[stop_id])) {
        reach_dat->stats.reach_dropped_stations_++;
        continue;
      }
    }
    

    if (stop_time.arrival_ < min) {
      //if (!reach_dat ||
      //    test_reach(reach_dat->reach_values, reach_dat->source_time_dep,
      //               stop_id, min, reach_dat->const_graph.dists_[stop_id])) {
        station_marks.mark(stop_id);
        current_round[stop_id] = stop_time.arrival_;
      //}
    }

    /*
     * The reason for the split in the update process for the current_round
     * and the earliest arrivals is that we might have some results in
     * current_round from former runs of the algorithm, but the earliest
     * arrivals start at invalid<time> every run.
     *
     * Therefore, we need to set the earliest arrival independently from
     * the results in current round.
     *
     * We cannot carry over the earliest arrivals from former runs, since
     * then we would skip on updates to the curren_round results.
     */

    if (stop_time.arrival_ < ea[stop_id]) {
      ea[stop_id] = stop_time.arrival_;
    }

    // check if we could catch an earlier trip
    auto const previous_k_arrival = prev_arrivals[stop_id];
    if (previous_k_arrival <= stop_time.departure_) {
      earliest_trip_id =
          std::min(earliest_trip_id,
                   get_earliest_trip(tt, route, prev_arrivals, r_stop_offset));
    }
  }
}

void update_footpaths(raptor_timetable const& tt, time* current_round,
                      earliest_arrivals const& ea,
                      cpu_mark_store& station_marks) {

  for (stop_id stop_id = 0; stop_id < tt.stop_count(); ++stop_id) {

    auto index_into_transfers = tt.stops_[stop_id].index_to_transfers_;
    auto next_index_into_transfers = tt.stops_[stop_id + 1].index_to_transfers_;

    for (auto current_index = index_into_transfers;
         current_index < next_index_into_transfers; ++current_index) {

      auto const& footpath = tt.footpaths_[current_index];

      if (!valid(ea[stop_id])) {
        continue;
      }

      // there is no triangle inequality in the footpath graph!
      // we cannot use the normal arrival values,
      // but need to use the earliest arrival values as read
      // and write to the normal arrivals,
      // otherwise it is possible that two footpaths
      // are chained together
      time const new_arrival = ea[stop_id] + footpath.duration_;

      time to_earliest_arrival = ea[footpath.to_];
      time to_arrival = current_round[footpath.to_];

      auto const min = std::min(to_arrival, to_earliest_arrival);
      if (new_arrival < min) {
        station_marks.mark(footpath.to_);
        current_round[footpath.to_] = new_arrival;
      }
    }
  }
}

reach_data::reach_data(raptor_meta_info const& meta_info,
                       raptor_query const& query, raptor_statistics & stats)
    : reach_values(meta_info.reach_values_),
      source_time_dep(query.source_time_end_),
      const_graph({meta_info.graph_, {query.target_}, {}}),
      stats(stats) {
  this->const_graph.run();
}

void invoke_cpu_raptor(schedule const& sched, raptor_query const& query,
                   raptor_meta_info const& meta_info, raptor_statistics & stats) {
  auto const& tt = query.tt_;

  auto& result = *query.result_;
  earliest_arrivals ea(tt.stop_count(), invalid<time>);

  cpu_mark_store station_marks(tt.stop_count());
  cpu_mark_store route_marks(tt.route_count());

  init_arrivals(result, query, station_marks);

  std::unique_ptr<reach_data> reach_data_p = nullptr;
  if (motis::raptor::use_reach) {
    MOTIS_START_TIMING(create_graph);
    reach_data_p = std::make_unique<reach_data>(meta_info, query, stats);
    stats.reach_graph_creation_time_ += MOTIS_GET_TIMING_MS(create_graph);
  }

  for (raptor_round round_k = 1; round_k < max_raptor_round; ++round_k) {
    bool any_marked = false;

    for (auto s_id = 0; s_id < tt.stop_count(); ++s_id) {

      if (!station_marks.marked(s_id)) {
        continue;
      }

      if (!any_marked) {
        any_marked = true;
      }

      auto const& stop = tt.stops_[s_id];
      for (auto sri = stop.index_to_stop_routes_;
           sri < stop.index_to_stop_routes_ + stop.route_count_; ++sri) {
        route_marks.mark(tt.stop_routes_[sri]);
      }
    }

    if (!any_marked) {
      break;
    }

    station_marks.reset();

    for (route_id r_id = 0; r_id < tt.route_count(); ++r_id) {
      if (!route_marks.marked(r_id)) {
        continue;
      }

      update_route(tt, r_id, result[round_k - 1], result[round_k], ea,
                   station_marks, reach_data_p.get());
    }

    route_marks.reset();

    update_footpaths(tt, result[round_k], ea, station_marks);
  }
}

struct CacheRequest {
  raptor_timetable const& timetable_;
  stop_id                 start_;
  time                    start_time_;
};


void find_best_routes(raptor_result & result, CacheRequest const& cache_request) {
  auto & timetable = cache_request.timetable_;
  earliest_arrivals ea(timetable.stop_count(), invalid<time>);

  cpu_mark_store station_marks(timetable.stop_count());
  cpu_mark_store route_marks(timetable.route_count());

  result[0][cache_request.start_] = cache_request.start_time_;
  station_marks.mark(cache_request.start_);

  for (raptor_round round_k = 1; round_k < max_raptor_round; ++round_k) {
    bool any_marked = false;

    for (auto stop_id = 0; stop_id < timetable.stop_count(); ++stop_id) {
      if (!station_marks.marked(stop_id)) {
        continue;
      }
      any_marked = true;

      auto const& stop = timetable.stops_[stop_id];
      for (auto i = 0; i < stop.route_count_; ++i) {
        route_marks.mark(timetable.stop_routes_[stop.index_to_stop_routes_ + i]);
      }
    }

    if (!any_marked) {
      break;
    }

    station_marks.reset();

    for (route_id r_id = 0; r_id < timetable.route_count(); ++r_id) {
      if (!route_marks.marked(r_id)) {
        continue;
      }

      update_route(timetable, r_id, result[round_k - 1], result[round_k], ea,
                   station_marks);
    }

    route_marks.reset();

    update_footpaths(timetable, result[round_k], ea, station_marks);
  }
}

void generate_reach_cache(raptor_timetable const& timetable,
                          std::vector<std::atomic<float>>& reach_values)
{
  logging::scoped_timer timer("generating reach cache] [cpu");

  CacheRequest cache_request {timetable, invalid<stop_id>, invalid<time>};
  raptor_result result(timetable.stop_count());
  std::unordered_set<time> processed_starts;

  auto bars = utl::global_progress_bars{false};

  auto tracker = utl::activate_progress_tracker("reach cache generation");
  tracker->status("processing nodes").out_bounds(0, 100).in_high(timetable.stop_count());
  auto skiped_tracker = utl::activate_progress_tracker("skipped duplicate starts");
  skiped_tracker->status("skipped departures").out_bounds(0, 100).in_high(timetable.stop_times_.size());

  //NOTE(Rennorb): we can't just loop over all routs and take their stops
  // directly as we might run the query for the same stop multiple times if
  // it's shared by multiple routes. which would be more wasteful that the
  // additional index resolutions we have to do just loop over all stops once.
  for(int src_stop = 0; src_stop < timetable.stop_count(); src_stop++)
  {
    //if(src_stop == 500) exit(0);
    //LOG(logging::info) << (float)src_stop / timetable.stop_count() << "% done";

    processed_starts.clear();

    auto source = timetable.stops_[src_stop];
    //LOG(logging::info) << "----------start: " << src_stop << "("<< source.route_count_ << " routes)------------";
    for(route_id route_idx : timetable.iterate_route_indices(source))
    {
      auto& route_for_times = timetable.routes_[route_idx];
      for(int route_stop = 0; route_stop < route_for_times.stop_count_; route_stop++)
      {
        auto stop_idx = timetable.route_stops_[route_for_times.index_to_route_stops_ + route_stop];

        if(stop_idx != src_stop) continue;
        //found the right stop

        for(int trip_for_times = 0; trip_for_times < route_for_times.trip_count_; trip_for_times++)
        {
          auto time_pair = timetable.stop_times_[
              route_for_times.index_to_stop_times_ + (trip_for_times * route_for_times.stop_count_) + route_stop];

          if (processed_starts.find(time_pair.departure_) != processed_starts.end()) {
            skiped_tracker->increment(1);
            continue;
          }
          processed_starts.insert(time_pair.departure_);

          cache_request.start_      = src_stop;
          cache_request.start_time_ = time_pair.departure_;

          result.reset();
          find_best_routes(result, cache_request);

          //TODO(Rennorb): could prob thread this
          for(int dst_stop = 0; dst_stop < timetable.stop_count(); dst_stop++)
          {
            // loop route (x to x)
            if(dst_stop == src_stop) continue;

            //TODO(Rennorb): just take the last one
            raptor_round best_arrival_round = invalid<raptor_round>;
            time         best_arrival_time  = invalid<time>;
            for(raptor_round round = 0; round < max_raptor_round; round++)
            {
              if(result[round][dst_stop] < best_arrival_time)
              {
                best_arrival_time  = result[round][dst_stop];
                best_arrival_round = round;
              }
            }
            // cant find a way to get here
            if(!valid(best_arrival_round)) continue;

            //start working backwards and reconstruct the journeys
            stop_id current_stop_idx = dst_stop;
            for(raptor_round round = best_arrival_round; round > 0; round--)
            {
              auto& current_stop = timetable.stops_[current_stop_idx];
              for(route_id current_stop_route_idx : timetable.iterate_route_indices(current_stop))
              {
                auto& route = timetable.routes_[current_stop_route_idx];
                stop_offset current_stop_offset = timetable.find_stop_offset(route, current_stop_idx);
                for(int current_stop_route_trip = 0; current_stop_route_trip < route.trip_count_; current_stop_route_trip++)
                {
                  auto time_idx = route.index_to_stop_times_ + (current_stop_route_trip * route.stop_count_) + current_stop_offset;
                  if(timetable.stop_times_[time_idx].arrival_ != result[round][current_stop_idx]) continue;
                  //found a trip that could be used

                  for(stop_offset offset = 0; offset < current_stop_offset; offset++)
                  {
                    time departure = timetable.get_time_at(route, current_stop_route_trip, offset).departure_;
                    auto entrance_stop_idx = timetable.get_route_stop_idx(route, offset);
                    /*if(valid(result[round - 1][entrance_stop_idx]))
                      LOG(logging::info) << format_time(result[round - 1][entrance_stop_idx]) << " --> "
                                         << entrance_stop_idx << " --> " << format_time(departure);
                                         */
                    if (departure < result[round - 1][entrance_stop_idx]) continue;
                    // found the entry point for this trip

                    //LOG(logging::info) << "found entrypoint at route "
                    //                << current_stop_route_idx << " and trip "
                    //                << current_stop_route_trip << " starting with "
                     //               << entrance_stop_idx << "@" << format_time(result[round - 1][entrance_stop_idx]);

                    auto update_reach = [&reach_values, &result, src_stop, best_arrival_time, round](stop_id node_idx) {
                      float reach_to_node = reach_metric(result[0][src_stop], result[round][node_idx]);
                      float reach_from_node = reach_metric(result[round][node_idx], best_arrival_time);
                      float new_reach = std::min(reach_to_node, reach_from_node);
                      
                      auto& slot = reach_values[node_idx];
                      float old;
                      do {
                        old = slot.load();
                        if (old >= new_reach) return;
                      } while (slot.compare_exchange_weak(old, new_reach));
                    };

                    // update all nodes on this trip
                    update_reach(current_stop_idx); // mark exit node
                    for(stop_offset to_mark = offset + 1; offset < current_stop_offset - 1; offset++)
                    {
                      auto node_idx = timetable.get_route_stop_idx(route, to_mark);
                      update_reach(node_idx);
                    }
                    update_reach(entrance_stop_idx); // mark entrance node

                    current_stop_idx = entrance_stop_idx;
                    goto multi_break;
                  }
                }
              }
              multi_break:;
            }
          }
        }
      }
    }
    tracker->increment(1);
  }
}

}  // namespace motis::raptor
