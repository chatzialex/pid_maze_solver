#pragma once

#include "pid_maze_solver/goals.hpp"
#include "yaml-cpp/yaml.h"
#include <algorithm>
#include <utility>

namespace YAML {

template <> struct convert<PidMazeSolver::Point2D> {
  static bool decode(const Node &node, PidMazeSolver::Point2D &rhs) {
    if (!node.IsMap() || node.size() != 2 || !node["x"] || !node["y"]) {
      return false;
    }

    rhs.x = node["x"].as<double>();
    rhs.y = node["y"].as<double>();

    return true;
  }
};

template <> struct convert<PidMazeSolver::GoalSequence> {
  static bool decode(const Node &node, PidMazeSolver::GoalSequence &rhs) {
    if (!node.IsSequence()) {
      return false;
    }

    PidMazeSolver::GoalSequence result{};

    for (const auto &subnode : node) {
      try {
        result.push_back(subnode.as<PidMazeSolver::Point2D>());
      } catch (YAML::BadConversion & /*e*/) {
        return false;
      }
    }

    rhs = std::move(result);
    return true;
  }
};

} // namespace YAML