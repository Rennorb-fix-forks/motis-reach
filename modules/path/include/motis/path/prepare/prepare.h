#pragma once

#include "conf/configuration.h"

namespace motis::path {

struct prepare_settings : public conf::configuration {
  prepare_settings() : configuration("Prepare Options", "") {
    param(schedules_, "schedules", "/path/to/rohdaten");
    param(prefixes_, "prefixes", "/path/to/rohdaten");
    param(osm_, "osm", "/path/to/germany-latest.osm.pbf");
    param(osrm_, "osrm", "path/to/osrm/files");
    param(out_, "out", "/path/to/db.mdb");
    param(tmp_, "tmp", "/path/to/tmp/directory");
    param(filter_, "filter", "filter station sequences");
    param(osm_cache_task_, "osm_cache_task", "{ignore, load, dump}");
    param(osm_cache_file_, "osm_cache_file", "/path/to/osm_cache.bin");
    param(seq_cache_task_, "seq_cache_task", "{ignore, load, dump}");
    param(seq_cache_file_, "seq_cache_file", "/path/to/seq_cache.fbs");
    param(db_size_, "db_size", "max db size");
  }

  std::vector<std::string> schedules_{"rohdaten"};
  std::vector<std::string> prefixes_{};
  std::string osm_{"germany-latest.osm.pbf"};
  std::string osrm_{"osrm"};
  std::string out_{"./pathdb.mdb"};
  std::string tmp_{"."};

  std::vector<std::string> filter_;

  std::string osm_cache_task_{"ignore"};
  std::string osm_cache_file_{"osm_cache.bin"};

  std::string seq_cache_task_{"ignore"};
  std::string seq_cache_file_{"seq_cache.bin"};

  size_t db_size_{sizeof(void*) >= 8
                      ? static_cast<size_t>(32) * 1024 * 1024 * 1024
                      : 256 * 1024 * 1024};
};

void prepare(prepare_settings const&);

}  // namespace motis::path
