#include "../../inc/lacam2/lacam2.hpp"

Solution solve(const LACAMInstance& ins, std::string& additional_info,
               const int verbose, const Deadline* deadline, std::mt19937* MT,
               const Objective objective, const float restart_rate)
{
  auto planner = Planner(&ins, deadline, MT, verbose, objective, restart_rate);
  return planner.solve(additional_info);
}
