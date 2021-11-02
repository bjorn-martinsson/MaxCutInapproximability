#pragma once

#include <cassert>
#include <map>

#include "config.h"
#include "edge.h"

bool isPowerOfTwo(long long x) {
  return (x & (x - 1)) == 0;
};

struct StructuredSet {
  size_t size = 0;
  long long affine_point = 0;
  std::vector<long long> basis;

 public:
  //StructuredSet() {
  //}
  long long reduce(long long node) {
    node ^= affine_point;
    for (auto b : basis)
      node = std::min(node ^ b, node);
    return node;
  }

  bool canBeAdded(long long node) {
    if (size == 0) return true;
    node = reduce(node);
    if (node == 0) return true;
    return isPowerOfTwo(size);
  }

  void add(long long node) {
    assert(canBeAdded(node));
    if (size == 0) {
      ++size;
      affine_point = node;
      return;
    }
    ++size;
    node = reduce(node);
    if (node) {
      std::reverse(basis.begin(), basis.end());
      basis.insert(std::lower_bound(basis.begin(), basis.end(), node), node);
      std::reverse(basis.begin(), basis.end());
    }
  }
};

std::vector<bool> getAllStructuredSets() {
  std::vector<bool> ret(n_nodes);
  for (long long node = 0; node < n_nodes; ++node) {
    StructuredSet set;
    long long bitmask = node;
    for (long long tmp = 0; tmp < dimension; ++tmp) {
      for (long long i = 0; i < dimension; ++i) {
        if (((bitmask >> i) & 1LL) && set.canBeAdded(i)) {
          set.add(i);
          bitmask ^= (1LL << i);
        }
      }
    }
    if (!bitmask)
      ret[node] = true;
  }
  return ret;
}
