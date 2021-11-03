#pragma once

#include <lemon/list_graph.h>
#include <lemon/preflow.h>

#include <cstdint>
#include <map>
#include <utility>
#include <vector>

#include "edge.h"
#include "gadget.h"
#include "node.h"
#include "orbit.h"
#include "partial_assignment.h"
#include "rational.h"

template <typename T>
class Evaluator {
  OrbitInfo orbitInfo;
  EdgeOrbitInfo edgeOrbitInfo;

 public:
  Evaluator(const OrbitInfo orbitInfo, const EdgeOrbitInfo edgeOrbitInfo) : orbitInfo(orbitInfo), edgeOrbitInfo(edgeOrbitInfo) {}

  PartialAssignment optimalRelaxedExtension(
      const PartialAssignment& partialAssignment, const Gadget<T>& gadget) {
    using lemon::ListDigraph;

    
    auto edgeOrbits = edgeOrbitInfo.edgeOrbits;


    // Construct nodes in the graph
    ListDigraph g;
    auto source = g.addNode();
    auto sink = g.addNode();
    
    std::map<long long, ListDigraph::Node> hypergraphNodes;
    for (auto edgeOrbit : edgeOrbits) {
      for (auto edge : edgeOrbit) {
        long long index1 = edge.a.getIndex();
        long long index2 = edge.b.getIndex();
        if (!hypergraphNodes.count(index1))
          hypergraphNodes[index1] = g.addNode();
        if (!hypergraphNodes.count(index2))
          hypergraphNodes[index2] = g.addNode();
      }
    }

    // Compute the capacity of each edge in the graph
    ListDigraph::ArcMap<T> capacity(g);
    T totalWeight = 0;
    
    for (auto edgeOrbit : edgeOrbits) {
      for (auto edge : edgeOrbit) {
        Node node = edge.a;
        Node destination = edge.b;
        auto arc1 = g.addArc(hypergraphNodes[node.getIndex()],
                            hypergraphNodes[destination.getIndex()]);
        auto arc2 = g.addArc(hypergraphNodes[destination.getIndex()],
                            hypergraphNodes[node.getIndex()]);
        T weight = gadget.getWeight(edgeOrbit[0]);
        capacity[arc1] = weight;
        capacity[arc2] = weight;
        totalWeight += weight;
      }
    }
 
    for (auto [index, hyperNode] : hypergraphNodes) {
      Node node((uint32_t)index);
      if (partialAssignment.isAssigned(node)) {
        ListDigraph::Arc arc;
        if (partialAssignment.getValue(node)) {
          arc = g.addArc(source, hyperNode);
        } else {
          arc = g.addArc(hyperNode, sink);
        }
        capacity[arc] = totalWeight;
      }
    }

    lemon::Preflow<ListDigraph, ListDigraph::ArcMap<T>> preflow(g, capacity, source,
                                                         sink);
    preflow.runMinCut();

    std::map<Node, bool> assignment;
    for (auto [index, hyperNode] : hypergraphNodes) {
      Node node((uint32_t)index);
      assignment[node] =
          preflow.minCut(hyperNode);
      if (partialAssignment.isAssigned(node)) {
        assert(partialAssignment.getValue(node) == assignment[node]);
      }
    }

    return PartialAssignment(assignment);
  }

  Rational<T> relaxedRandomCost(const Gadget<T>& gadget) {
    T totalValue = 0;
    auto allOrbits = orbitInfo.nodeOrbits;
    for (const auto& orbit : allOrbits) {
      Node representative = orbit[0];
      std::map<Node, bool> assignment;
      for (uint32_t S = 0; S < dimension; S++) {
        assignment[chi(S)] = (representative[S] == 1);
        assignment[-chi(S)] = !assignment[chi(S)];
      }

      auto extendedAssignment =
          optimalRelaxedExtension(PartialAssignment(assignment), gadget);
      T value = 0;

      for (auto edgeOrbit : edgeOrbitInfo.edgeOrbits) {
        for (auto edge : edgeOrbit) {
          Node node = edge.a;
          Node destination = edge.b;
          if (extendedAssignment.getValue(node) !=
              extendedAssignment.getValue(destination)) {
            value += gadget.getWeight(Edge(edgeOrbit[0].a, edgeOrbit[0].b));
          }
        }
      }

      totalValue += orbit.size() * value;
    }
    return Rational<T>(totalValue, n_nodes);
  }
};
