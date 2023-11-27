/*
 * instance definition
 */
#pragma once
#include <random>

#include "graph.hpp"
#include "utils.hpp"

struct LACAMInstance {
  const Graph G;
  Config starts;
  Config goals;
  const uint N;  // number of agents

  // for testing
  LACAMInstance(const std::string& map_filename,
           const std::vector<int>& start_indexes,
           const std::vector<int>& goal_indexes);
  // for MAPF benchmark
  LACAMInstance(const std::string& scen_filename, const std::string& map_filename,
           const uint _N = 1);
  // random instance generation
  LACAMInstance(const std::string& map_filename, std::mt19937* MT,
           const uint _N = 1);
  ~LACAMInstance() {}

  // simple feasibility check of instance
  bool is_valid(const int verbose = 0) const;
};

// solution: a sequence of configurations
using Solution = std::vector<Config>;
std::ostream& operator<<(std::ostream& os, const Solution& solution);
