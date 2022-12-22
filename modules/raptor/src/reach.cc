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

}  // namespace motis::raptor
