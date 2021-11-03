#include <boost/multiprecision/cpp_int.hpp>

using int_type = boost::multiprecision::cpp_int;

#include <iostream>
#include <vector>

#include "structured.h"
#include "config.h"
#include "gadget.h"
#include "optimizer.h"
#include "orbit.h"
#include "rational.h"

int main() {
  std::cout << "Finding all structured sets" << std::endl;
  std::vector<bool> structuredSets = getAllStructuredSets();
  long long i = 0;
  for (long long ind = 0; ind < n_nodes; ++ind)
    i += structuredSets[ind];
  std::cout << "Found " << i << " nodes to consider" << std::endl;
  
  std::cout << "Analyzing orbits of the symmetry group..." << std::endl;
  OrbitInfo orbitInfo;

  EdgeOrbitInfo edgeOrbitInfo(orbitInfo, structuredSets);
  Optimizer<int_type> optimizer;
  Evaluator<int_type> evaluator(orbitInfo, edgeOrbitInfo);

  std::cout << "Optimizing gadget..." << std::endl;

  auto gadget = optimizer.gadgetSearch(orbitInfo, edgeOrbitInfo);

  std::cout << "Finished optimizing gadget" << std::endl;

  auto randomCost = evaluator.relaxedRandomCost(gadget);
 
  int_type totalWeight = 0;
  for (auto edgeOrbit : edgeOrbitInfo.edgeOrbits)
    totalWeight += edgeOrbit.size() * gadget.getWeight(edgeOrbit[0]);
  auto dictCost = Rational<int_type>(totalWeight, dimension);

  auto inapproximabilityFactor = randomCost / dictCost;

  std::cout << "The found gadget has inapproximability factor "
            << inapproximabilityFactor.a << "/" << inapproximabilityFactor.b
            << std::endl;
}
