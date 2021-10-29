#pragma once

#include <array>
#include <cassert>
#include <map>

#include "config.h"
#include "edge.h"
#include "node.h"

class PartialAssignment {
 public:
  PartialAssignment(std::map<Node, bool> assignment) {
    for (long long i = 0; i < n_nodes; i++) {
      assigned[i] = false;
    }
    for (const auto& node : assignment) {
      assigned[node.first.getIndex()] = true;
      value[node.first.getIndex()] = node.second;
    }
  }

  bool isAssigned(const Node& node) const { return assigned[node.getIndex()]; }

  bool getValue(const Node& node) const {
    assert(assigned[node.getIndex()]);
    return value[node.getIndex()];
  }

 private:
  // assigned[x] specifies if node x is assigned
  std::array<bool, n_nodes> assigned;

  // value[x] specifies if node x is assigned
  std::array<bool, n_nodes> value;
};
