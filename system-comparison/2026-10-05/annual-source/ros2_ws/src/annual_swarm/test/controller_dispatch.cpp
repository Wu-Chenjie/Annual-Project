#include "controller.hpp"
#include <cmath>
#include <iostream>
int main() {
  sim::BacksteppingController backstep;
  backstep.set_use_smc(false);
  sim::Controller* base=&backstep;
  std::array<double,12> state{}; state[2]=1.;
  const auto output=base->compute_control(state,{0,0,1.1});
  // Correct integral backstepping derivative: alpha1_dot includes -K0*z1.
  const double expected=9.81 + .1*(3.*2.2+1.+.6)+3.*.6*.001;
  if (std::abs(output[0]-expected)>1e-9) {
    std::cerr << "Backstepping position loop not dispatched or derivative incorrect: " << output[0] << '\n';
    return 1;
  }
  base->reset(); backstep.K1[2]=1.;
  const auto changed=base->compute_control(state,{0,0,1.1});
  if (std::abs(changed[0]-output[0])<.1) return 2;
  base->reset(); state[2]=1.1;
  if (std::abs(base->compute_control(state,{0,0,1.1})[0]-9.81)>1e-9) return 3;
  return 0;
}
