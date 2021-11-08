#pragma once

#include <algorithm>
#include <utility>

#include "node.h"
#include "orbit.h"

struct Edge {
  Node a;
  Node b;

  Edge() : a(), b() {}

  Edge(Node a, Node b) : a(a), b(b) {}

  // Define less than operator to allow storing edges in ordered trees
  bool operator<(const Edge& other) const {
    // Since the edges are undirected we first sort the nodes in each edge
    auto myNodes = std::make_pair(a.getIndex(), b.getIndex());
    if (myNodes.second < myNodes.first)
      std::swap(myNodes.first, myNodes.second);

    auto otherNodes = std::make_pair(other.a.getIndex(), other.b.getIndex());
    if (otherNodes.second < otherNodes.first)
      std::swap(otherNodes.first, otherNodes.second);

    // Compare lexicographically
    return myNodes < otherNodes;
  }

  bool operator==(const Edge& other) const {
    return (a == other.a && b == other.b) || (a == other.b && b == other.a);
  }
};

struct EdgeOrbitInfo {
  std::vector<Edge> allEdges;
  std::vector<std::vector<Edge>> edgeOrbits;
  std::vector<bool> usedNodes;
  OrbitInfo orbitInfo;

  void calcAllEdges() {
    for (long long index = 0; index < n_nodes; index++) {
      Node node((uint32_t)index);
      for (unsigned direction = 0; direction < dimension; direction++) {
        auto destination = node.getNeighbour(direction);
        if (node < destination && usedNodes[node.getIndex()] && usedNodes[destination.getIndex()])
          allEdges.emplace_back(node, destination);
      }
    }
  }

  void calcEdgeOrbits() {
    std::map<std::pair<uint8_t,uint8_t>, std::vector<Edge>> allOrbits;
    for (auto edge : allEdges) {
      auto node = edge.a;
      auto destination = edge.b;
      auto type1 = orbitInfo.getOrbit(node).nodeType;
      auto type2 = orbitInfo.getOrbit(destination).nodeType;
      if (type1 > type2)
        std::swap(type1, type2);
      allOrbits[{type1, type2}].push_back(edge);
    }
    
    for (auto [typepair, edgeOrbit] : allOrbits)
      edgeOrbits.push_back(edgeOrbit);
  }

  EdgeOrbitInfo(OrbitInfo orbitInfo, std::vector<bool> usedNodes) : orbitInfo(orbitInfo), usedNodes(usedNodes) {
    for (auto &nodeOrbit : orbitInfo.nodeOrbits) {
      auto val = usedNodes[nodeOrbit.getRepresentative().getIndex()];
      for (auto node : nodeOrbit.getAllNodes()) {
        assert(usedNodes[node.getIndex()] == val);
      }
    }

    calcAllEdges();
    calcEdgeOrbits();
  }

  std::vector<Edge> getOrbit(Edge representative) {
    uint8_t atype = orbitInfo.getOrbit(representative.a).nodeType;
    uint8_t btype = orbitInfo.getOrbit(representative.b).nodeType;

    for (auto &edgeOrbit : edgeOrbits) {
      uint8_t atype2 = orbitInfo.getOrbit(edgeOrbit[0].a).nodeType;
      uint8_t btype2 = orbitInfo.getOrbit(edgeOrbit[0].b).nodeType;
      if ((atype == atype2 && btype == btype2) || (atype == btype2 && btype == atype2))
        return edgeOrbit;
    }
    assert(false);
  }
};
