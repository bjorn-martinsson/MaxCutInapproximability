#pragma once

#include <array>
#include <cassert>
#include <map>

#include "config.h"
#include "edge.h"
#include "node.h"

class PartialAssignment {
 private:
  // assigned[x] specifies if node x is assigned
  std::map<Node, bool> assignment;
 
 public:
  PartialAssignment(std::map<Node, bool> assignment) : assignment(assignment) {};

  bool isAssigned(Node node) {return assignment.count(node);}

  bool getValue(Node node) {
    assert(isAssigned(node));
    return assignment[node];
  }

};
