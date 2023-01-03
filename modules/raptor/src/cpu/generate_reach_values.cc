#include <thread>
#include <unordered_set>

#include "motis/raptor/raptor.h"
#include "motis/raptor/cpu/cpu_raptor.h"
#include "motis/raptor/reconstructor.h"
#include "utl/progress_tracker.h"
#include "motis/core/common/logging.h"
#include "motis/core/common/timing.h"

namespace motis::raptor {

struct WorkerTask {
  stop_id source_stop_id;
  time    source_stop_time;
};

void stop_workers();
void init_workers(std::vector<atomic_reach> & reach_values, ReachGenerationData const& request);
void find_best_routes(raptor_result & result, WorkerTask const& task, raptor_timetable const& timetable);
void worker_thread(std::vector<atomic_reach> & reach_values, ReachGenerationData const& request);

std::vector<std::thread> workers;
std::mutex               requests_mutex;
std::condition_variable  requests_waiter;
std::queue<WorkerTask>   requests;
bool                     terminate_workers = false;
std::atomic<uint32_t>    processed_jobs;

void generate_reach_values(ReachGenerationData & request) {
  logging::scoped_timer timer("generating reach cache] [cpu");

  auto& [_, timetable, meta_info] = request;

  static_assert(sizeof(atomic_reach) == sizeof(reach_t));
  assert(meta_info.reach_values_.size() == timetable.stop_count());
  std::vector<atomic_reach> reach_values((size_t)timetable.stop_count(), atomic_reach{0});

  init_workers(reach_values, request);

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
  workers.emplace_back(worker_thread, std::ref(reach_values), std::ref(request));

  while (processed_jobs != gen_counter) {
    processed_tracker->status(
        fmt::format("{} / {}", processed_jobs, gen_counter));
    processed_tracker->update(processed_jobs);
    std::this_thread::sleep_for(std::chrono::seconds(10));
  }

  stop_workers();

  memcpy(meta_info.reach_values_.data(), reach_values.data(), sizeof(reach_t) * reach_values.size());
}

void init_workers(std::vector<atomic_reach> & reach_values, ReachGenerationData const& request) {
  processed_jobs = 0;
  auto worker_cnt = std::thread::hardware_concurrency();
  workers.reserve(worker_cnt);
  for (int i = 1; i < worker_cnt; i++)
    workers.emplace_back(worker_thread, std::ref(reach_values), std::ref(request));
  LOG(logging::info) << "started " << worker_cnt << " workers (1 with delayed start later on)";
}

void stop_workers() {
  terminate_workers = true;
  requests_waiter.notify_all();
  for (auto& worker : workers) {
    if(worker.joinable()) worker.join();
  }
}

void worker_thread(std::vector<atomic_reach> & reach_values, ReachGenerationData const& request) {
  auto& [schedule, timetable, meta_info] = request;

  raptor_result result{timetable.stop_count()};
  reconstructor reconstructor{schedule, meta_info, timetable};

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
    for (int dst_stop = 0; dst_stop < timetable.stop_count(); dst_stop++) {
      // loop route (x to x)
      if (dst_stop == task.source_stop_id) continue;

      reconstructor.journeys_.clear();

      reach_query query {task.source_stop_id, task.source_stop_time, dst_stop, result};
      reconstructor.add(query);

      //NOTE(Rennorb): this is basically `reconstructor.get_journeys()`
      // but without some irrelevant stuff and while still keeping the station_ids
      utl::erase_if(reconstructor.journeys_, [&](auto const& ij) {
        return ij.get_duration() > max_travel_duration;
      });

      /*NOTE(Rennorb): usually we would have to reverse the stops. 
         We are just going to not do this as we dont realyl care about the order.
      for(auto& journey : reconstructor.journeys_) {
        std::reverse(begin(journey.stops_), end(journey.stops_));
      }
      */

      for(auto& journey : reconstructor.journeys_) {
        for(auto& stop : journey.stops_) {
          float reach_to_node = reach_metric(task.source_stop_time, stop.a_sched_time_);
          float reach_from_node = reach_metric(stop.d_sched_time_, journey.get_arrival());
          reach_t new_reach = std::min(reach_to_node, reach_from_node);

          auto& slot = reach_values[stop.station_id_];
          reach_t old;
          do {
            old = slot.load();
            if (old >= new_reach) break;
          } while (slot.compare_exchange_weak(old, new_reach));
        }
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
