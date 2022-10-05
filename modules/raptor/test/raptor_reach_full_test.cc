#include <numeric>
#include "gtest/gtest.h"

#include "motis/test/schedule/simple_realtime.h"
#include "motis/test/motis_instance_test.h"

#include "motis/loader/loader.h"
#include "motis/raptor/raptor.h"
#include "motis/raptor/get_raptor_timetable.h"
#include "motis/raptor/raptor_result.h"
#include "motis/module/message.h"
#include "motis/core/journey/journey.h"
#include "motis/core/journey/message_to_journeys.h"

using namespace flatbuffers;
using namespace motis;
using namespace motis::test;
using namespace motis::raptor;
using namespace motis::module;
using namespace motis::routing;

loader::loader_options options_full = {
    .dataset_ = { "../motis-test-data/berlin-gtfs" },
    .schedule_begin_ = "20210808",
    .num_days_ = 2
};

struct raptor_reach_full_test : public motis_instance_test {
  raptor_reach_full_test()
      : motis::test::motis_instance_test(options_full,
                                         {"routing", "raptor"}) {}



  msg_ptr make_routing_request(std::string const& target) {
    auto const interval = Interval(unix_time(1300), unix_time(1300));
    message_creator fbb;
    fbb.create_and_finish(
        MsgContent_RoutingRequest,
        CreateRoutingRequest(
            fbb, motis::routing::Start_PretripStart,
            CreatePretripStart(
                fbb,
                CreateInputStation(fbb, fbb.CreateString("070101006383"),
                                   fbb.CreateString("")),
                &interval)
                .Union(),
            CreateInputStation(fbb, fbb.CreateString("070101007458"),
                               fbb.CreateString("")),
            SearchType_Default, SearchDir_Forward,
            fbb.CreateVector(std::vector<Offset<Via>>()),
            fbb.CreateVector(std::vector<Offset<AdditionalEdgeWrapper>>()))
            .Union(),
        target);
    return make_msg(fbb);
  }
};

TEST_F(raptor_reach_full_test, reach_raptor_full_test) {
  auto const sched = loader::load_schedule(options_full);

  int runs = 10;
  std::vector<uint64_t> with_reach(runs);
  std::vector<uint64_t> without_reach(runs);

  const char* timer = "raptor_time"; //total_calculation_time

  for(int i = 0; i < runs; i++) {
    motis::raptor::use_reach = true;
    auto response = call(make_routing_request("/raptor_cpu"));
    auto result = motis_content(RoutingResponse, response);
    auto with = result->statistics()
            ->LookupByKey("raptor")
            ->entries()->LookupByKey(timer)
            ->value();
    with_reach[i] = with;

    motis::raptor::use_reach = false;
    auto response2 = call(make_routing_request("/raptor_cpu"));
    auto result2 = motis_content(RoutingResponse, response2);
    auto without = result2->statistics()->LookupByKey("raptor")
                    ->entries()->LookupByKey(timer)
                    ->value();
    without_reach[i] = without;

    LOG(logging::info) << "run #" << i << ": with reach: " << with
                       << "without reach: " << without;

    auto testee = message_to_journeys(result);
    auto reference = message_to_journeys(result2);
    EXPECT_EQ(reference, testee);
  }

  auto avg_with    = (float)std::reduce(with_reach.begin(), with_reach.end()) / (float)runs;
  auto avg_without = (float)std::reduce(without_reach.begin(), without_reach.end()) / (float)runs;

  LOG(logging::info) << runs << " runs; with reach: " << avg_with
                             << ", without reach: " << avg_without;

}


