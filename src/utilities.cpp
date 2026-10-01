#include "utilities.hpp"
#include <fstream>

void write_fix_csv(const std::filesystem::path &path, const int dim,
                   const std::vector<int> &fix_steps,
                   const std::vector<Eigen::VectorXd> &measurements,
                   const std::vector<Eigen::VectorXd> &innovations,
                   const std::vector<Eigen::MatrixXd> &S) {
  std::ofstream out(path);

  out << "timestep";
  for (int i = 0; i < dim; i++) {
    out << ",z_" << i;
  }
  for (int i = 0; i < dim; i++) {
    out << ",innovation_" << i;
  }
  for (int r = 0; r < dim; r++) {
    for (int c = 0; c < dim; c++) {
      out << ",S_" << r << "_" << c;
    }
  }
  out << "\n";

  for (size_t i = 0; i < fix_steps.size(); i++) {
    out << fix_steps[i];
    for (int j = 0; j < dim; j++) {
      out << "," << measurements[i](j);
    }
    for (int j = 0; j < dim; j++) {
      out << "," << innovations[i](j);
    }
    for (int r = 0; r < dim; r++) {
      for (int c = 0; c < dim; c++) {
        out << "," << S[i](r, c);
      }
    }
    out << "\n";
  }
}
