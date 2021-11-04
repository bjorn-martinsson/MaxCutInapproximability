#pragma once

#include <cassert>
#include <cstdint>
#include <map>
#include <set>
#include <vector>

#include "config.h"
#include "node.h"

Node chi(uint32_t S) {
  uint32_t z = 0;
  for (unsigned i = 0; i < k; i++) {
    if (S & (1 << i)) {
      for (unsigned j = 0; j < dimension; j++) {
        if (j & (1 << i)) {
          z ^= 1u << j;
        }
      }
    }
  }
  return Node(z);
}

struct LinearBasis {
  std::vector<uint32_t> basis;
  
  uint32_t reduce(uint32_t node) {
    for (auto b : basis)
      node = std::min(b ^ node, node);
    return node;
  }

  void add(uint32_t node) {
    node = reduce(node);
    if (node) {
      auto pos = std::lower_bound(basis.begin(), basis.end(), node, std::greater());
      basis.insert(pos, node);
    }
  }
};


struct OrbitInfo {
  LinearBasis linearBasis;
  std::set<uint32_t> nodeRepresentatives;

  uint8_t numNodeTypes;
  std::vector<uint8_t> nodeType;
  
  std::vector<Node> allNodes;
  std::vector<std::vector<Node>> nodeOrbits;

  void splitBasedOnType(int checkType) {
    // Computes how many neighbours of the checked type a given node has
    auto numberOfNeighboursOfType = [&](uint32_t node) {
      int cnt = 0;
      for (int j = 0; j < dimension; j++) {
        uint32_t neighbour = linearBasis.reduce(node ^ (1u << j));
        if (nodeType[neighbour] == checkType) {
          cnt++;
        }
      }
      return cnt;
    };

    // Compute how many neighbours of the checked type that nodes of different
    // types have
    std::vector<std::set<uint8_t>> seenNeighbourCounts(numNodeTypes);
    std::vector<uint8_t> seenRepresentativeCounts((*nodeRepresentatives.rbegin()) + 1);
    for (auto i : nodeRepresentatives) {
      auto seen = numberOfNeighboursOfType((uint32_t)i);
      seenNeighbourCounts[nodeType[i]].insert(seen);
      seenRepresentativeCounts[i] = seen;
    }

    // Compute what node type a node should get based on how many neighbours it
    // has of the checked type
    std::vector<std::map<int, uint8_t>> countToNewNodeType(numNodeTypes);
    for (size_t type = 0; type < countToNewNodeType.size(); type++) {
      auto it = seenNeighbourCounts[type].begin();
      countToNewNodeType[type][*it] = (uint8_t)type;
      for (it++; it != seenNeighbourCounts[type].end(); it++) {
        countToNewNodeType[type][*it] = numNodeTypes;
        numNodeTypes++;
        assert(numNodeTypes != 0);
      }
    }

    // Update the node types
    for (auto i : nodeRepresentatives) {
      nodeType[i] = countToNewNodeType[nodeType[i]][seenRepresentativeCounts[i]];
    }
  }


  void calcAllNodes() {
    for (long long index = 0; index < n_nodes; index++) {
      allNodes.emplace_back(index);
    }
  }
  
  void calcNodeOrbits() {
    nodeOrbits.resize(numNodeTypes);
    for (size_t node = 0; node < nodeType.size(); node++) {
      nodeOrbits[nodeType[node]].emplace_back(node);
    }
  }

  OrbitInfo() {
    for (uint32_t S = 0; S < dimension; ++S) {
      linearBasis.add(chi(S).getIndex());
      linearBasis.add((-chi(S)).getIndex());
    }

    for (long long node = 0; node < n_nodes; ++node) {
      nodeRepresentatives.insert(linearBasis.reduce(node));
    }

    std::cout << "Max size of node representative " << *std::max_element(nodeRepresentatives.begin(), nodeRepresentatives.end()) << std::endl;

    std::cout << "Number of node representatives " << nodeRepresentatives.size() << std::endl;

    // Start by splitting the set of nodes based on whether they belong to Z
    numNodeTypes = 2;
    nodeType = std::vector<uint8_t>((*nodeRepresentatives.rbegin()) + 1, 1);
    nodeType[0] = 0;

    for (int checkType = 0; checkType < numNodeTypes; ++checkType) {
      // Split sets of nodes based on how many neighbours they have of type
      // checkType
      std::cout << "Checking type " << checkType << " out of " << (long long)numNodeTypes << std::endl;
      splitBasedOnType(checkType); 
    }
    
    std::cout << "Orbits found: " << (long long)numNodeTypes << std::endl;

    std::vector<uint8_t>fullNodeType(n_nodes);

    for (uint32_t S = 0; S < dimension; ++S) {
      uint32_t base = chi(S).getIndex();
      uint32_t base2 = (-chi(S)).getIndex();
      for (auto node : nodeRepresentatives) {
        fullNodeType[node ^ base] = nodeType[node];
        fullNodeType[node ^ base2] = nodeType[node];
      }
    }
    nodeType = move(fullNodeType);

    calcAllNodes();
    calcNodeOrbits();

    //std::cout << "Orbits found " << nodeOrbits.size() << std::endl;
    //for (auto orbit : nodeOrbits)
    //std::cout << "Orbit size " << orbit.size() << std::endl;
  }

  std::vector<Node> getOrbit(const Node& representative) const {
    uint8_t type = nodeType[representative.getIndex()];
    
    for (auto nodeOrbit : nodeOrbits)
      if (nodeType[nodeOrbit[0].getIndex()] == type)
        return nodeOrbit;
    assert(false);
  }
};
