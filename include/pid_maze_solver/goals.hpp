#pragma once

#include <optional>
#include <vector>

namespace PidMazeSolver {

struct Point2D {
  double x;
  double y;
};

using GoalSequence = std::vector<Point2D>;

} // namespace PidMazeSolver