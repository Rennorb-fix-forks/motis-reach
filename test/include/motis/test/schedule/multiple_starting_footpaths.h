#pragma once

#include "motis/module/message.h"
#include "motis/loader/loader_options.h"

namespace motis {

struct schedule;

namespace test::schedule::multiple_starting_foothpaths {

static auto const dataset_opt = loader::loader_options{
    .dataset_ = {"test/schedule/multiple_starting_footpaths"},
    .schedule_begin_ = "20151124"};

}  // namespace test::schedule::multiple_starting_foothpaths
}  // namespace motis
