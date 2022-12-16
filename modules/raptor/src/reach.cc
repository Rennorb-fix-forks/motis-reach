#include <atomic>

#include "motis/raptor/reach.h"
#include "motis/raptor/raptor_query.h"
#include "motis/raptor/raptor_statistics.h"
#include "motis/raptor/raptor_timetable.h"

namespace motis::raptor {

reach_wrapper::reach_wrapper(reach_t val) : std::atomic<float>(val) {}
reach_wrapper::reach_wrapper(reach_wrapper const& other) : std::atomic<float>(other.load()) {}
reach_wrapper& reach_wrapper::operator=(reach_wrapper const& other) {
  this->store(other.load());
  return *this;
}

reach_data::reach_data(raptor_meta_info const& meta_info,
                       raptor_query const& query, raptor_statistics& stats,
                       mcd::vector<motis::station_ptr> const& stations)
    : reach_values_(meta_info.reach_values_),
      source_time_dep_(query.source_time_end_),
      stats_(stats),
      stations_(stations) {}

}  // namespace motis::raptor
