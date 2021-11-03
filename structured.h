#pragma once

#include <cassert>
#include <map>

#include "config.h"
#include "edge.h"
#include "orbit.h"

bool isPowerOfTwo(long long x) {
  return (x & (x - 1)) == 0;
};

struct StructuredSet {
  size_t size = 0;
  long long affine_point = 0;
  std::vector<long long> basis;

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
      auto pos = std::lower_bound(basis.begin(), basis.end(), node, std::greater());
      basis.insert(pos, node);
    }
  }
};

bool isStructured(long long bitmask) {
  long long bits = 0;
  for (long long i = 0; i < dimension; ++i) {
    bits += ((bitmask >> i) & 1LL);
  }
  if (bits > dimension/4)
    return false;

  StructuredSet set;
  for (long long tmp = 0; tmp < dimension; ++tmp) {
    for (long long i = 0; i < dimension; ++i) {
      if (((bitmask >> i) & 1LL) && set.canBeAdded(i)) {
        set.add(i);
        bitmask ^= (1LL << i);
      }
    }
  }
  return bitmask == 0;
}

std::vector<bool> getAllStructuredSets() {
  std::vector<bool> ret(n_nodes);
  long long count = 0;
  ret[0] = true;
  for (long long node = 0; node < n_nodes; ++node) {
    ret[node] = ret[node] && isStructured(node);
    if (ret[node]) {
      ++count;
      for (long long i = 0; i < dimension; ++i) {
        long long destination = node ^ (1LL << i);
        if (destination > node)
          ret[destination] = true;
      }
    }
  }
  
  std::cout << "Found " << count << " structured sets" << std::endl;

  for (long long i = 0; i < k; ++i) {
    long long base = chi(1LL << i).getIndex();
    for (long long node = 0; node < n_nodes; ++node) {
      ret[node ^ base] = ret[node ^ base] || ret[node];
    }
  }

  for (long long node = 0; node < n_nodes; ++node) {
    ret[node] = ret[node] || ret[(-Node(node)).getIndex()];
  }

  return ret;
}
