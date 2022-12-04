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
  struct Data {
    std::vector<uint64_t> timer;
    std::vector<uint64_t> queries;
    std::vector<uint64_t> dropped_queries;

    Data(int runs) : timer(runs), queries(runs), dropped_queries(runs) {}
  } with_reach(runs), without_reach(runs);

  struct Result {
    msg_ptr response; //keep alive
    RoutingResponse const* result;

    ~Result() = default;
  };

  auto const call_and_update_stats = [&](int run_index, Data& data) -> Result {
    auto response = call(make_routing_request("/raptor_cpu"));
    auto result = motis_content(RoutingResponse, response);
    auto raptor_stats = result->statistics()->LookupByKey("raptor");

    auto timer_field = raptor_stats->entries()->LookupByKey("raptor_time"); //total_calculation_time
    if (timer_field) data.timer[run_index] = timer_field->value();

    auto queries_field = raptor_stats->entries()->LookupByKey("raptor_station_queries");
    if (queries_field) data.queries[run_index] = queries_field->value();

    auto dropped_queries_field = raptor_stats->entries()->LookupByKey("reach_dropped_stations");
    if (dropped_queries_field) data.dropped_queries[run_index] = dropped_queries_field->value();

    return { response, result };
  };

  for(int i = 0; i < runs; i++) {
    motis::raptor::use_reach = true;
    auto result1 = call_and_update_stats(i, with_reach);

    motis::raptor::use_reach = false;
    auto result2 = call_and_update_stats(i, without_reach);

    auto reference = message_to_journeys(result2.result);
    auto testee = message_to_journeys(result1.result);
    EXPECT_EQ(reference, testee);
  }

  auto avg_time_with    = (float)std::reduce(with_reach.timer.begin(), with_reach.timer.end()) / runs;
  auto avg_time_without = (float)std::reduce(without_reach.timer.begin(), without_reach.timer.end()) / runs;

  auto avg_queries_with    = (float)std::reduce(with_reach.queries.begin(), with_reach.queries.end()) / runs;
  auto avg_queries_without = (float)std::reduce(without_reach.queries.begin(), without_reach.queries.end()) / runs;

  auto avg_dropped_queries_with    = (float)std::reduce(with_reach.dropped_queries.begin(), with_reach.dropped_queries.end()) / runs;
  auto avg_dropped_queries_without = (float)std::reduce(without_reach.dropped_queries.begin(), without_reach.dropped_queries.end()) / runs;

  LOG(logging::info) << runs << " runs \n" 
                     << "time           : with reach : " << avg_time_with << ", without reach: " << avg_time_without << "\n" 
                     << "queries        : with reach : " << avg_queries_with << ", without reach: " << avg_queries_without << "\n" 
                     << "dropped queries: with reach : " << avg_dropped_queries_with << ", without reach: " << avg_dropped_queries_without;

}


