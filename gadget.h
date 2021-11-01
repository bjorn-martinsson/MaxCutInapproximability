#pragma once

#include <cassert>
#include <map>

#include "config.h"
#include "edge.h"

template <typename T>
class Gadget {
  std::map<Edge, T> weight;
  //T totalWeight;

 public:
  Gadget(std::map<Edge, T> weight) : weight(weight) {
    //totalWeight = 0;
    //for (const auto& it : weight) {
    //  totalWeight += it.second;
    //}
  }

  T getWeight(const Edge& edge) const {
    auto it = weight.find(edge);
    assert(it != weight.end());
    return it->second;
  }

  //T getTotalWeight() const { return totalWeight; }

  bool operator==(const Gadget& other) { return weight == other.weight; }
};
