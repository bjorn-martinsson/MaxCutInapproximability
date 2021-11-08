#pragma once

#include <iostream>
#include <map>
#include <vector>
#include <chrono>
#include <random>

#include "config.h"
#include "edge.h"
#include "evaluator.h"
#include "gadget.h"
#include "node.h"
#include "orbit.h"
#include "simplex_rational.h"

template <typename T>
class Optimizer {
 public:
  Gadget<T> gadgetSearch(OrbitInfo& orbitInfo, EdgeOrbitInfo& edgeOrbitInfo) {
    Evaluator<T> evaluator(orbitInfo, edgeOrbitInfo);

    using U = Rational<T>;

    auto &edgeOrbits = edgeOrbitInfo.edgeOrbits;
    auto &nodeOrbits = orbitInfo.nodeOrbits;

    // Construct initial linear program
    std::vector<std::vector<U>> A;
    std::vector<U> b, c;

    // The first entries in the vector represent the gadget's weights on
    // different edge orbits
    for (auto &edgeOrbit : edgeOrbits) {
      c.emplace_back(0);
    }
    // The next entries in the vector represent the gadget's relaxed costs for
    // different Z-assignment orbits
    for (auto &nodeOrbit : nodeOrbits) {
      c.emplace_back(nodeOrbit.size(), n_nodes);
    }

    A.emplace_back();
    for (auto &edgeOrbit : edgeOrbits) {
      A[0].emplace_back(edgeOrbit.size());
    }
    for (auto &nodeOrbit : nodeOrbits) {
      A[0].emplace_back(0);
    }
    b.emplace_back(1);

    A.emplace_back();
    for (auto &edgeOrbit : edgeOrbits) {
      A[1].emplace_back(-((U)edgeOrbit.size()));
    }
    for (auto &nodeOrbit : nodeOrbits) {
      A[1].emplace_back(0);
    }
    b.emplace_back(-1);

    // Construct initial gadget
    std::map<Edge, T> uniformGadgetWeights;
    for (auto &edgeOrbit : edgeOrbits) {
      uniformGadgetWeights[edgeOrbit[0]] = 1;
    }
    Gadget<T> gadget(uniformGadgetWeights);

    std::vector<Gadget<T>> gadgetSet;
    U lower, upper, totalGadgetWeight, bestLower{0}, bestUpper{100};

    // Improve gadget iteratively
    while (std::find(gadgetSet.begin(), gadgetSet.end(), gadget) ==
           gadgetSet.end()) {

      gadgetSet.push_back(gadget);
      std::cout << "Iteration " << gadgetSet.size() << std::endl;
      
      std::cout << "Progress of iteration: ";
      double progress = 0;

      lower = 0;
      // Compute optimal extensions for all orbits of Z-assignments
      for (auto &nodeOrbit : nodeOrbits) {
        Node representative = nodeOrbit.getRepresentative();
        std::map<Node, bool> assignment;
        for (uint32_t S = 0; S < dimension; S++) {
          assignment[chi(S)] = (representative[S] == 1);
          assignment[-chi(S)] = !assignment[chi(S)];
        }

        auto extendedAssignment =
            evaluator.optimalRelaxedExtension(PartialAssignment(assignment), gadget);

        progress += 1.0/nodeOrbits.size();
        std::cout << 100 * progress << "% ";

        // Add constraint corresponding to the extended assignment
        A.emplace_back();
        for (size_t i = 0; i < edgeOrbits.size(); i++) {
          auto &edgeOrbit = edgeOrbits[i];
          A.back().emplace_back(0);
          for (auto edge : edgeOrbit) {
            if (extendedAssignment.getValue(edge.a) !=
                extendedAssignment.getValue(edge.b)) {
              A.back().back() -= 1;
            }
          }
          lower -= A.back().back() * nodeOrbit.size() * gadget.getWeight(edgeOrbit[0]);
        }
        for (auto &nodeOrbit2 : nodeOrbits) {
          A.back().emplace_back(nodeOrbit2.nodeType == nodeOrbit.nodeType);
        }
        b.emplace_back(0);
      }
      std::cout << std::endl;

      // Solve linear program to find a new gadget
      LPSolver<T> solver(A, b, c);

      std::vector<U> solution;
      solver.solve(solution);

      upper = 0;
      for (size_t i = edgeOrbits.size(); i < edgeOrbits.size() + nodeOrbits.size(); ++i)
        upper += solution[i] * c[i];
      upper *= dimension;

      totalGadgetWeight = 0;
      for (auto &edgeOrbit : edgeOrbits)
        totalGadgetWeight += edgeOrbit.size() * gadget.getWeight(edgeOrbit[0]);

      lower /= n_nodes; lower /= totalGadgetWeight; lower *= dimension;
      std::cout << "Result from iteration: lower = " << lower << ", upper = " << upper << std::endl;
      
      bestLower = max(lower, bestLower);
      bestUpper = min(upper, bestUpper);
      std::cout << "Inapproximability factor between: " << bestLower << " and " << bestUpper << std::endl;

      T commonDenominator = 1;
      for (size_t i = 0; i < edgeOrbits.size(); i++) {
        commonDenominator = lcm(commonDenominator, solution[i].b);
      }

      std::map<Edge, T> gadgetWeights;
      for (size_t i = 0; i < edgeOrbits.size(); i++) {
        assert(commonDenominator % solution[i].b == 0);
        T weight = solution[i].a * (commonDenominator / solution[i].b);
        gadgetWeights[edgeOrbits[i][0]] = weight;
      }
      gadget = Gadget<T>(gadgetWeights);
    }

    return gadget;
  }
};
