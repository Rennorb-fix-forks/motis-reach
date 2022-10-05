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

using motis::test::schedule::simple_realtime::dataset_opt;

struct raptor_reach_test : public motis_instance_test {
  raptor_reach_test()
      : motis::test::motis_instance_test(dataset_opt, {"routing", "raptor"}) {}

  msg_ptr make_routing_request(std::string const& target) {
    message_creator fbb;
    fbb.create_and_finish(
        MsgContent_RoutingRequest,
        CreateRoutingRequest(
            fbb, motis::routing::Start_OntripStationStart,
            CreateOntripStationStart(
                fbb,
                CreateInputStation(fbb, fbb.CreateString("8000096"),
                                   fbb.CreateString("")),
                unix_time(1300))
                .Union(),
            CreateInputStation(fbb, fbb.CreateString("8000080"),
                               fbb.CreateString("")),
            SearchType_Default, SearchDir_Forward,
            fbb.CreateVector(std::vector<Offset<Via>>()),
            fbb.CreateVector(std::vector<Offset<AdditionalEdgeWrapper>>()))
            .Union(),
        target);
    return make_msg(fbb);
  }
};

TEST_F(raptor_reach_test, reach_raptor_init) {
  auto const sched = loader::load_schedule(dataset_opt);

  int runs = 100;
  std::vector<uint64_t> with_reach(runs);
  std::vector<uint64_t> without_reach(runs);

  const char* timer = "raptor_time"; //total_calculation_time

  for(int i = 0; i < runs; i++) {
    motis::raptor::use_reach = true;
    auto response = call(make_routing_request("/raptor_cpu"));
    auto result = motis_content(RoutingResponse, response);
    with_reach[i] = result->statistics()->LookupByKey("raptor")
                     ->entries()->LookupByKey(timer)
                      ->value();
    

    motis::raptor::use_reach = false;
    auto response2 = call(make_routing_request("/raptor_cpu"));
    auto result2 = motis_content(RoutingResponse, response2);
    without_reach[i] = result2->statistics()->LookupByKey("raptor")
                    ->entries()->LookupByKey(timer)
                    ->value();

    auto testee = message_to_journeys(result);
    auto reference = message_to_journeys(result2);
    EXPECT_EQ(reference, testee);
  }

  auto avg_with    = (float)std::reduce(with_reach.begin(), with_reach.end()) / (float)runs;
  auto avg_without = (float)std::reduce(without_reach.begin(), without_reach.end()) / (float)runs;

  LOG(logging::info) << runs << " runs; with reach: " << avg_with
                             << ", without reach: " << avg_without;

}


