#include <boost/multiprecision/cpp_int.hpp>

using int_type = boost::multiprecision::cpp_int;

#include <iostream>
#include <vector>

#include "config.h"
#include "gadget.h"
#include "optimizer.h"
#include "orbit.h"
#include "rational.h"

int main() {
  std::cout << "Analyzing orbits of the symmetry group..." << std::endl;

  OrbitInfo orbitInfo;
  Optimizer<int_type> optimizer;
  Evaluator<int_type> evaluator(orbitInfo);

  std::cout << "Optimizing gadget..." << std::endl;

  auto gadget = optimizer.gadgetSearch(orbitInfo);

  std::cout << "Finished optimizing gadget" << std::endl;

  auto randomCost = evaluator.relaxedRandomCost(gadget);
  auto dictCost = Rational<int_type>(gadget.getTotalWeight(), dimension);

  auto inapproximabilityFactor = randomCost / dictCost;

  std::cout << "The found gadget has inapproximability factor "
            << inapproximabilityFactor.a << "/" << inapproximabilityFactor.b
            << std::endl;
}
