#pragma once

#include <cassert>
#include <cstdint>
#include <map>
#include <set>
#include <vector>

#include "config.h"
#include "edge.h"
#include "node.h"

class OrbitInfo {
  uint8_t numNodeTypes;
  std::vector<uint8_t> nodeType;
  
  std::vector<Node> allNodes;
  std::vector<Edge> allEdges;
  std::vector<std::vector<Node>> nodeOrbits;
  std::vector<std::vector<Edge>> edgeOrbits;

  void splitBasedOnType(int checkType) {
    // Computes how many neighbours of the checked type a given node has
    auto numberOfNeighboursOfType = [checkType, this](uint32_t node) {
      int cnt = 0;
      for (int j = 0; j < dimension; j++) {
        uint32_t neighbour = node ^ (1u << j);
        if (nodeType[neighbour] == checkType) {
          cnt++;
        }
      }
      return cnt;
    };

    // Compute how many neighbours of the checked type that nodes of different
    // types have
    std::vector<std::set<int>> seenNeighbourCounts(numNodeTypes);
    for (size_t i = 0; i < nodeType.size(); i++) {
      seenNeighbourCounts[nodeType[i]].insert(
          numberOfNeighboursOfType((uint32_t)i));
    }

    // Compute what node type a node should get based on how many neighbours it
    // has of the checked type
    std::vector<std::map<int, uint8_t>> countToNewNodeType(numNodeTypes);
    for (size_t type = 0; type < countToNewNodeType.size(); type++) {
      auto it = seenNeighbourCounts[type].begin();
      countToNewNodeType[type][*it] = (int8_t)type;
      for (it++; it != seenNeighbourCounts[type].end(); it++) {
        countToNewNodeType[type][*it] = numNodeTypes;
        numNodeTypes++;
      }
    }

    std::vector<uint8_t> newNodeType(nodeType.size());

    // Update the node types
    for (size_t i = 0; i < nodeType.size(); i++) {
      newNodeType[i] =
          countToNewNodeType[nodeType[i]]
                            [numberOfNeighboursOfType((uint32_t)i)];
    }

    nodeType = move(newNodeType);
  }


  void calcAllNodes() {
    for (long long index = 0; index < n_nodes; index++) {
      allNodes.emplace_back(index);
    }
  }

  void calcAllEdges() {
    for (long long index = 0; index < n_nodes; index++) {
      Node node((uint32_t)index);
      for (unsigned direction = 0; direction < dimension; direction++) {
        auto destination = node.getNeighbour(direction);
        if (node < destination)
          allEdges.emplace_back(node, destination);
      }
    }
  }

  void calcNodeOrbits() {
    nodeOrbits.resize(numNodeTypes);
    for (size_t node = 0; node < nodeType.size(); node++) {
      nodeOrbits[nodeType[node]].emplace_back(node);
    }
  }

  void calcEdgeOrbits() {
    std::map<std::pair<uint8_t,uint8_t>, std::vector<Edge>> allOrbits;
    for (auto edge : allEdges) {
      auto node = edge.a;
      auto destination = edge.b;
      auto type1 = nodeType[node.getIndex()];
      auto type2 = nodeType[destination.getIndex()];
      if (type1 > type2)
        std::swap(type1, type2);
      allOrbits[{type1, type2}].push_back(edge);
    }
    
    for (auto [typepair, edgeOrbit] : allOrbits)
      edgeOrbits.push_back(edgeOrbit);
  }




 public:
  Node chi(uint32_t S) const {
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

  OrbitInfo() {
    // Start by splitting the set of nodes based on whether they belong to Z
    numNodeTypes = 2;
    nodeType = std::vector<uint8_t>(n_nodes, 1);
    for (uint32_t S = 0; S < dimension; S++) {
      nodeType[chi(S).getIndex()] = 0;
      nodeType[(-chi(S)).getIndex()] = 0;
    }

    for (int checkType = 0; checkType < numNodeTypes; ++checkType) {
      // Split sets of nodes based on how many neighbours they have of type
      // checkType
      splitBasedOnType(checkType);  
    }

    calcAllNodes();
    calcAllEdges();
    calcNodeOrbits();
    calcEdgeOrbits();
  }

  std::vector<Node> getAllNodes() const {
    return allNodes;
  }

  std::vector<Edge> getAllEdges() const {
    return allEdges;
  }

  std::vector<std::vector<Node>> getAllNodeOrbits() const {
    return nodeOrbits;
  }

  std::vector<std::vector<Edge>> getAllEdgeOrbits() const {
    return edgeOrbits;
  }

  std::vector<Node> getOrbit(const Node& representative) const {
    uint8_t type = nodeType[representative.getIndex()];
    
    for (auto nodeOrbit : nodeOrbits)
      if (nodeType[nodeOrbit[0].getIndex()] == type)
        return nodeOrbit;
    assert(false);
  }

  std::vector<Edge> getOrbit(const Edge& representative) const {
    uint8_t atype = nodeType[representative.a.getIndex()];
    uint8_t btype = nodeType[representative.b.getIndex()];

    for (auto edgeOrbit : edgeOrbits) {
      uint8_t atype2 = nodeType[edgeOrbit[0].a.getIndex()];
      uint8_t btype2 = nodeType[edgeOrbit[0].b.getIndex()];
      if ((atype == atype2 && btype == btype2) || (atype == btype2 && btype == atype2))
        return edgeOrbit;
    }
    assert(false);
  }
};
