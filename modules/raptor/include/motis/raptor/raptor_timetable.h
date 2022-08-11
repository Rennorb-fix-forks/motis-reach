#pragma once

#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include "utl/verify.h"

#include "motis/core/schedule/connection.h"
#include "motis/core/schedule/time.h"

namespace motis::raptor {

using time8 = uint8_t;

using stop_id = int32_t;
using route_id = uint32_t;
using footpath_id = int32_t;
using trip_id = uint32_t;

using motis_id = int32_t;

// these are only 16bit wide, because they are used relativ to a station/route
// i.e. how many stops/trips a single route has
//      how many routes/footpaths a single station has
using trip_count = uint16_t;
using stop_count = uint16_t;
using route_count = uint16_t;
using footpath_count = uint16_t;

using stop_offset = uint16_t;

using stop_times_index = uint32_t;
using route_stops_index = uint32_t;
using stop_routes_index = uint32_t;
using footpaths_index = uint32_t;

using raptor_round = uint8_t;
using transfers = uint8_t;

using earliest_arrivals = std::vector<time>;

template <typename T>
inline constexpr T min_value = std::numeric_limits<T>::min();

template <typename T>
inline constexpr T max_value = std::numeric_limits<T>::max();

template <typename T>
inline constexpr T invalid = max_value<T>;
// Template specializations in raptor_timetable.cc

template <typename T>
inline constexpr auto valid(T const& value) {
  return value != invalid<T>;
}

constexpr raptor_round max_transfers = 6;
constexpr raptor_round max_trips = max_transfers + 1;
constexpr raptor_round max_raptor_round = max_trips + 1;

constexpr time const max_travel_duration = 1440;

struct raptor_stop {
  raptor_stop() = delete;
  raptor_stop(footpath_count const fc, route_count const rc,
              footpaths_index const it, stop_routes_index const isr)
      : footpath_count_{fc},
        route_count_{rc},
        index_to_transfers_{it},
        index_to_stop_routes_{isr} {}

  footpath_count footpath_count_;
  route_count route_count_;
  footpaths_index index_to_transfers_;
  stop_routes_index index_to_stop_routes_;
};

struct stop_time {
  time arrival_{invalid<decltype(arrival_)>};
  time departure_{invalid<decltype(departure_)>};
};

struct raptor_route {
  raptor_route() = delete;
  raptor_route(trip_count const tc, stop_count const sc,
               stop_times_index const sti, route_stops_index const rsi)
      : trip_count_{tc},
        stop_count_{sc},
        index_to_stop_times_{sti},
        index_to_route_stops_{rsi} {}

  trip_count trip_count_;
  stop_count stop_count_;
  stop_times_index index_to_stop_times_;
  route_stops_index index_to_route_stops_;
};


struct raptor_footpath {
  raptor_footpath() = delete;
  raptor_footpath(stop_id const to, time const duration)
      : to_{to}, duration_{static_cast<time8>(duration)} {
    utl::verify(duration < std::numeric_limits<time8>::max(),
                "Footpath duration too long to fit inside time8");
  }
  stop_id to_ : 24;
  time8 duration_;
};

struct raptor_incoming_footpath {
  raptor_incoming_footpath() = delete;
  raptor_incoming_footpath(stop_id const from, time const duration)
      : from_{from}, duration_{static_cast<time8>(duration)} {
    utl::verify(duration < std::numeric_limits<time8>::max(),
                "Footpath duration too long to fit inside time8");
  }
  stop_id from_ : 24;
  time8 duration_;
};

struct raptor_timetable {
  raptor_timetable() = default;
  raptor_timetable(raptor_timetable const&) = default;
  ~raptor_timetable() = default;

  raptor_timetable& operator=(raptor_timetable const&) = delete;
  raptor_timetable(raptor_timetable&&) = delete;
  raptor_timetable& operator=(raptor_timetable const&&) = delete;

  std::vector<raptor_stop> stops_;
  std::vector<raptor_route> routes_;
  std::vector<raptor_footpath> footpaths_;

  std::vector<stop_time> stop_times_;
  std::vector<stop_id> route_stops_;
  std::vector<route_id> stop_routes_;

  // Needed for the reconstruction
  // duration REDUCED by the transfer times from the departure station
  std::vector<std::vector<raptor_incoming_footpath>> incoming_footpaths_;

  auto stop_count() const {  // subtract the sentinel
    return static_cast<stop_id>(stops_.size() - 1);
  }

  auto route_count() const {  // subtract the sentinel
    return static_cast<route_id>(routes_.size() - 1);
  }

  auto footpath_count() const {
    return static_cast<footpath_id>(footpaths_.size());
  }

  [[nodiscard]] inline stop_offset
  find_stop_offset(raptor_route const& route, stop_id stop_idx) const
  {
    for(stop_offset offset = 0; offset < route.stop_count_; offset++)
      if(this->route_stops_[route.index_to_route_stops_ + offset] == stop_idx)
        return offset;
    return invalid<stop_offset>;
  }

  template<typename T>
  struct index_iterator {
    T const* start_;
    T const* end_;

    index_iterator() = delete;
    index_iterator(T const* start, T const* end)
    {
      start_ = start;
      end_   = end;
    }

    T const* begin() { return start_; }
    T const* end() { return end_; }
  };

  [[nodiscard]] inline index_iterator<route_id>
  iterate_route_indices(raptor_stop const& stop) const
  {
    auto start = stop.index_to_stop_routes_;
    return {this->stop_routes_.data() + start,
            this->stop_routes_.data() + start + stop.route_count_};
  }

  [[nodiscard]] inline stop_time const&
  get_time_at(raptor_route const& route, trip_id _trip_id,
              stop_offset _stop_offset) const
  {
    auto idx = route.index_to_stop_times_ + (route.stop_count_ * _trip_id)
               + _stop_offset;
    return this->stop_times_[idx];
  }

  [[nodiscard]] inline stop_id const&
  get_route_stop_idx(raptor_route const& route, stop_offset _stop_offset) const
  {
    return this->route_stops_[route.index_to_route_stops_ + _stop_offset];
  }
};

struct raptor_meta_info {
  raptor_meta_info() = default;
  raptor_meta_info(raptor_meta_info const&) = delete;
  raptor_meta_info& operator=(raptor_meta_info const&) = delete;
  raptor_meta_info(raptor_meta_info&&) = delete;
  raptor_meta_info& operator=(raptor_meta_info const&&) = delete;
  ~raptor_meta_info() = default;

  std::unordered_map<std::string, stop_id> eva_to_raptor_id_;
  std::vector<std::string> raptor_id_to_eva_;
  std::vector<unsigned> station_id_to_index_;
  std::vector<time> transfer_times_;

  // contains the stop_id itself as first element
  std::vector<std::vector<stop_id>> equivalent_stations_;

  // for every station the departure events of the station
  // and the stations reachable by foot
  std::vector<std::vector<time>> departure_events_;

  // for every station the departure events of the station
  // and the stations reachable by foot
  // for all meta stations
  std::vector<std::vector<time>> departure_events_with_metas_;

  // uses the same indexing scheme as the stop times vector in the
  // timetable, but the first entry for every trip is a nullptr! since
  // #stop_times(trip) = #lcons(trip) + 1
  std::vector<light_connection const*> lcon_ptr_;

  // duration of the footpaths INCLUDE transfer time from the departure
  // station
  std::vector<std::vector<raptor_footpath>> initialization_footpaths_;

  //NOTE(Rennorb): Be s,v,t vertices of a graph G and m(x, y, G) a metric.
  // The reach of a v on a G is the max{ (s,t) : min(m(s,v,G), m(v,t,G)) } where
  // (x,y) is a least cost path from x to y in G
  std::vector<float> reach_values_;
};

}  // namespace motis::raptor
