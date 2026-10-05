// The C++ force law (include/multi_mode_controller/utils/force_law.h) against ForceVAM's Python law: replays the
// inputs of every recorded step (traces from ForceVAM's control/export_law_traces.py, one file per case) and checks
// the commanded target, stiffness and damping ratio match. No ROS needed:
//   g++ -O2 -std=c++17 -I include -I /usr/include/eigen3 test/force_law_test.cpp -o /tmp/force_law_test
//   /tmp/force_law_test test/traces/*.txt
#include <multi_mode_controller/utils/force_law.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using panda_controllers::ForceLaw;
using panda_controllers::ForceLawParams;

static ForceLawParams parseParams(const std::string& line) {
  std::istringstream in(line);
  std::map<std::string, double> kv;
  std::string key, value;
  while (in >> key >> value) kv[key] = std::strtod(value.c_str(), nullptr);
  ForceLawParams p;
  p.k_rest = kv.at("k_rest");
  p.k_load = kv.at("k_load");
  p.zeta_rest = kv.at("zeta_rest");
  p.zeta_load = kv.at("zeta_load");
  p.lead = kv.at("lead");
  p.give = kv.at("give");
  p.guard = kv.at("guard");
  p.tau = kv.at("tau");
  p.push = kv.at("push");
  p.push_max = kv.at("push_max");
  p.k_max = kv.at("k_max");
  p.zeta_max = kv.at("zeta_max");
  p.tank = kv.at("tank") > 0.5;
  return p;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "usage: force_law_test <trace.txt> ...\n";
    return 2;
  }
  const double tol = 1e-6;
  bool all_ok = true;
  for (int f = 1; f < argc; ++f) {
    std::ifstream in(argv[f]);
    std::string line;
    std::getline(in, line);
    ForceLaw law(parseParams(line));
    size_t steps = 0;
    double worst = 0.0;
    size_t worst_step = 0;
    while (std::getline(in, line)) {
      std::istringstream row(line);
      std::vector<double> v;
      double x;
      while (row >> x) v.push_back(x);
      if (v.size() != 28) {
        std::cerr << argv[f] << ": a line with " << v.size() << " numbers (28 expected)\n";
        return 2;
      }
      Eigen::Vector3d planned(v[0], v[1], v[2]), hand(v[3], v[4], v[5]), force(v[6], v[7], v[8]);
      Eigen::Matrix3d rot;
      rot << v[9], v[10], v[11], v[12], v[13], v[14], v[15], v[16], v[17];
      Eigen::Vector3d target, k, z;
      law.step(planned, hand, force, rot, v[18], target, k, z);
      // relative to the size of each output: targets in m, stiffness in N/m, ratios
      const double err = std::max({(target - Eigen::Vector3d(v[19], v[20], v[21])).cwiseAbs().maxCoeff(),
                                   (k - Eigen::Vector3d(v[22], v[23], v[24])).cwiseAbs().maxCoeff() / 1000.0,
                                   (z - Eigen::Vector3d(v[25], v[26], v[27])).cwiseAbs().maxCoeff()});
      if (err > worst) {
        worst = err;
        worst_step = steps;
      }
      ++steps;
    }
    const bool ok = worst < tol;
    all_ok = all_ok && ok;
    std::printf("%s %s: %zu steps, worst difference %.3g (step %zu)\n", ok ? "OK  " : "FAIL", argv[f], steps, worst,
                worst_step);
  }
  return all_ok ? 0 : 1;
}
