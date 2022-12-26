#include <thread>
#include <unordered_set>

#include "motis/raptor/raptor.h"
#include "motis/raptor/cpu/cpu_raptor.h"
#include "utl/progress_tracker.h"
#include "motis/core/common/logging.h"
#include "motis/core/common/timing.h"

namespace motis::raptor {

struct WorkerTask {
  stop_id source_stop_id;
  time    source_stop_time;
};

void stop_workers();
void init_workers(std::vector<atomic_reach>& reach_storage, raptor_timetable const& timetable);
void find_best_routes(raptor_result & result, WorkerTask const& task, raptor_timetable const& timetable);
void worker_thread(std::vector<atomic_reach> & reach_values, raptor_timetable const& timetable);

std::vector<std::thread> workers;
std::mutex               requests_mutex;
std::condition_variable  requests_waiter;
std::queue<WorkerTask>   requests;
bool                     terminate_workers = false;
std::atomic<uint32_t>    processed_jobs;

void generate_reach_values(
  std::vector<reach_t>          & reach_value_storage,
  raptor_timetable         const& timetable,
  mcd::vector<station_ptr> const& stations
) {
  logging::scoped_timer timer("generating reach cache] [cpu");

  static_assert(sizeof(atomic_reach) == sizeof(reach_t));
  assert(reach_value_storage.size() == timetable.stop_count());
  std::vector<atomic_reach> reach_values((size_t)timetable.stop_count(), atomic_reach{0});

  init_workers(reach_values, timetable);

  auto bars = utl::global_progress_bars{false};

  auto gen_tracker =
      utl::activate_progress_tracker("reach cache job generation");
  gen_tracker->status("generating jobs")
      .out_bounds(0, 100)
      .in_high(timetable.stop_count());
  auto processed_tracker =
      utl::activate_progress_tracker("processed cache requests");
  processed_tracker->status("requests")
      .out_bounds(0, 100)
      .in_high(timetable.stop_count());
  int gen_counter = 0, skip_counter = 0;

  std::unordered_set<time> processed_starts;
  // NOTE(Rennorb): we can't just loop over all routs and take their stops
  //  directly as we might run the query for the same stop multiple times if
  //  it's shared by multiple routes. which would be more wasteful that the
  //  additional index resolutions we have to do just loop over all stops once.
  for (int src_stop = 0; src_stop < timetable.stop_count(); src_stop++) {
    processed_starts.clear();

    auto source = timetable.stops_[src_stop];
    for (route_id route_idx : timetable.iterate_route_indices(source)) {
      auto& route_for_times = timetable.routes_[route_idx];
      for (int route_stop = 0; route_stop < route_for_times.stop_count_;
           route_stop++) {
        auto stop_idx =
            timetable.route_stops_[route_for_times.index_to_route_stops_ +
                                   route_stop];

        if (stop_idx != src_stop) continue;
        // found the right stop

        for (int trip_for_times = 0;
             trip_for_times < route_for_times.trip_count_; trip_for_times++) {
          auto time_pair =
              timetable
                  .stop_times_[route_for_times.index_to_stop_times_ +
                               (trip_for_times * route_for_times.stop_count_) +
                               route_stop];

          if (processed_starts.find(time_pair.departure_) !=
              processed_starts.end()) {
            skip_counter++;
            continue;
          }
          processed_starts.insert(time_pair.departure_);

          requests.push({src_stop, time_pair.departure_});
          requests_waiter.notify_one();

          gen_counter++;
        }
      }
    }

    gen_tracker->increment(1);
    int jobs_estimate = gen_counter / gen_tracker->in_ * timetable.stop_count();
    if (jobs_estimate > processed_tracker->in_high_)
      processed_tracker->in_high_ = jobs_estimate;
    processed_tracker->update(processed_jobs);
  }

  gen_tracker
      ->status(fmt::format(
          "generation finished {}% dpartures with duplicated times skipped",
          skip_counter * 100.0f / (skip_counter + gen_counter)))
      .show_progress(false);
  processed_tracker->in_high_ = gen_counter;

  // push back the final thread that was used for creating tasks so far
  workers.emplace_back(worker_thread, std::ref(reach_values),
                       std::ref(timetable));

  while (processed_jobs != gen_counter) {
    processed_tracker->status(
        fmt::format("{} / {}", processed_jobs, gen_counter));
    processed_tracker->update(processed_jobs);
    std::this_thread::sleep_for(std::chrono::seconds(10));
  }

  stop_workers();

  memcpy(reach_value_storage.data(), reach_values.data(), sizeof(reach_t) * reach_values.size());
}

void init_workers(std::vector<atomic_reach> & reach_values, raptor_timetable const& timetable) {
  processed_jobs = 0;
  auto worker_cnt = std::thread::hardware_concurrency();
  workers.reserve(worker_cnt);
  for (int i = 1; i < worker_cnt; i++)
    workers.emplace_back(worker_thread, std::ref(reach_values), std::ref(timetable));
  LOG(logging::info) << "started " << worker_cnt << " workers (1 with delayed start later on)";
}

void stop_workers() {
  terminate_workers = true;
  requests_waiter.notify_all();
  for (auto& worker : workers) {
    if(worker.joinable()) worker.join();
  }
}

void worker_thread(std::vector<atomic_reach> & reach_values, raptor_timetable const& timetable) {
  raptor_result result{timetable.stop_count()};
  while (true) {
    WorkerTask task;
    {
      std::unique_lock lock(requests_mutex);
      requests_waiter.wait(lock, [] { return !requests.empty() || terminate_workers; });
      if (terminate_workers) return;

      task = requests.front();
      requests.pop();
    }

    result.reset();
    find_best_routes(result, task, timetable);

    // TODO(Rennorb): could prob thread this
    for (int dst_stop = 0; dst_stop < timetable.stop_count();
      dst_stop++) {
      // loop route (x to x)
      if (dst_stop == task.source_stop_id) continue;

      // TODO(Rennorb): just take the last one - maybe reverse or remove this 
      raptor_round best_arrival_round = invalid<raptor_round>;
      time best_arrival_time = invalid<time>;
      for (raptor_round round = 0; round < max_raptor_round; round++) {
        if (result[round][dst_stop] < best_arrival_time) {
          best_arrival_time = result[round][dst_stop];
          best_arrival_round = round;
        }
      }
      // cant find a way to get here
      if (!valid(best_arrival_round)) continue;

      // start working backwards and reconstruct the journeys
      stop_id current_stop_idx = dst_stop;
      for (raptor_round round = best_arrival_round; round > 0; round--) {
        auto& current_stop = timetable.stops_[current_stop_idx];

        for (route_id current_stop_route_idx : timetable.iterate_route_indices(current_stop)) {
          auto& route = timetable.routes_[current_stop_route_idx];
          stop_offset current_stop_offset = timetable.find_stop_offset(route, current_stop_idx);

          for (int current_stop_route_trip = 0; current_stop_route_trip < route.trip_count_; current_stop_route_trip++) {
            auto time_idx = route.index_to_stop_times_ + (current_stop_route_trip * route.stop_count_) + current_stop_offset;

            if (timetable.stop_times_[time_idx].arrival_ != result[round][current_stop_idx])
              continue;
            // found a trip that could be used

            for (stop_offset offset = 0; offset < current_stop_offset; offset++) {
              time departure = timetable.get_time_at(route, current_stop_route_trip, offset).departure_;
              auto entrance_stop_idx = timetable.get_route_stop_idx(route, offset);
              if (departure < result[round - 1][entrance_stop_idx]) continue;
              // found the entry point for this trip

              auto update_reach = [&](stop_id curr) {
                float reach_to_node = reach_metric(task.source_stop_time, result[round][curr]);
                float reach_from_node = reach_metric(result[round][curr], result[round][dst_stop]);
                reach_t new_reach = std::min(reach_to_node, reach_from_node);

                auto& slot = reach_values[curr];
                reach_t old;
                do {
                  old = slot.load();
                  if (old >= new_reach) break;
                } while (slot.compare_exchange_weak(old, new_reach));
              };

              // update all nodes on this trip
              update_reach(current_stop_idx);  // mark exit node
              for (stop_offset to_mark = offset + 1;
                offset < current_stop_offset - 1; offset++) {
                auto node_idx = timetable.get_route_stop_idx(route, to_mark);
                update_reach(node_idx);
              }
              update_reach(entrance_stop_idx);  // mark entrance node

              current_stop_idx = entrance_stop_idx;
              goto multi_break;
            }
          }
        }
      multi_break:;
      }
    }

    processed_jobs.fetch_add(1, std::memory_order_relaxed);
  }
}

void find_best_routes(raptor_result & result, WorkerTask const& task, raptor_timetable const& timetable) {
  earliest_arrivals ea(timetable.stop_count(), invalid<time>);

  cpu_mark_store station_marks(timetable.stop_count());
  cpu_mark_store route_marks(timetable.route_count());

  result[0][task.source_stop_id] = task.source_stop_time;
  station_marks.mark(task.source_stop_id);

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

}
