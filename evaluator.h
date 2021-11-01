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

 public:
  Evaluator(const OrbitInfo orbitInfo) : orbitInfo(orbitInfo) {}

  PartialAssignment optimalRelaxedExtension(
      const PartialAssignment& partialAssignment, const Gadget<T>& gadget) {
    using lemon::ListDigraph;

    // Construct nodes in the graph
    ListDigraph g;
    auto source = g.addNode();
    auto sink = g.addNode();
    std::vector<ListDigraph::Node> hypergraphNodes;
    for (long long index = 0; index < n_nodes; index++) {
      hypergraphNodes.emplace_back(g.addNode());
    }

    // Compute the capacity of each edge in the graph
    ListDigraph::ArcMap<T> capacity(g);
    T totalWeight = 0;
    
    for (auto edgeOrbit : orbitInfo.getAllEdgeOrbits()) {
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
  
    for (long long index = 0; index < n_nodes; index++) {
      Node node((uint32_t)index);
      if (partialAssignment.isAssigned(node)) {
        ListDigraph::Arc arc;
        if (partialAssignment.getValue(node)) {
          arc = g.addArc(source, hypergraphNodes[node.getIndex()]);
        } else {
          arc = g.addArc(hypergraphNodes[node.getIndex()], sink);
        }
        capacity[arc] = totalWeight;
      }
    }
    //for (long long index = 0; index < n_nodes; index++) {
    //  Node node((uint32_t)index);
    //  for (unsigned direction = 0; direction < dimension; direction++) {
    //    auto destination = node.getNeighbour(direction);
    //    auto arc = g.addArc(hypergraphNodes[node.getIndex()],
    //                        hypergraphNodes[destination.getIndex()]);
    //    T weight = gadget.getWeight(Edge(node, destination));
    //    capacity[arc] = weight;
    //  }

    //  if (partialAssignment.isAssigned(node)) {
    //    ListDigraph::Arc arc;
    //    if (partialAssignment.getValue(node)) {
    //      arc = g.addArc(source, hypergraphNodes[node.getIndex()]);
    //    } else {
    //      arc = g.addArc(hypergraphNodes[node.getIndex()], sink);
    //    }
    //    capacity[arc] = gadget.getTotalWeight();
    //  }
    //}

    lemon::Preflow<ListDigraph, ListDigraph::ArcMap<T>> preflow(g, capacity, source,
                                                         sink);
    preflow.runMinCut();

    std::map<Node, bool> assignment;
    for (long long index = 0; index < n_nodes; index++) {
      Node node((uint32_t)index);
      assignment[Node((uint32_t)index)] =
          preflow.minCut(hypergraphNodes[index]);
      if (partialAssignment.isAssigned(node)) {
        assert(partialAssignment.getValue(node) == assignment[node]);
      }
    }

    return PartialAssignment(assignment);
  }

  Rational<T> relaxedRandomCost(const Gadget<T>& gadget) {
    T totalValue = 0;
    auto allOrbits = orbitInfo.getAllNodeOrbits();
    for (const auto& orbit : allOrbits) {
      Node representative = orbit[0];
      std::map<Node, bool> assignment;
      for (uint32_t S = 0; S < dimension; S++) {
        assignment[orbitInfo.chi(S)] = (representative[S] == 1);
        assignment[-orbitInfo.chi(S)] = !assignment[orbitInfo.chi(S)];
      }

      auto extendedAssignment =
          optimalRelaxedExtension(PartialAssignment(assignment), gadget);
      T value = 0;

      for (auto edgeOrbit : orbitInfo.getAllEdgeOrbits()) {
        for (auto edge : edgeOrbit) {
      //for (long long index = 0; index < n_nodes; index++) {
        //Node node((uint32_t)index);
        //for (unsigned direction = 0; direction < dimension; direction++) {
          //auto destination = node.getNeighbour(direction);
          Node node = edge.a;
          Node destination = edge.b;
          //if (extendedAssignment.getValue(node) >
          //    extendedAssignment.getValue(destination)) {
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
