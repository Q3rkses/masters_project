#include "results.hpp"
#include <fstream>

namespace {

void write_vector_header(std::ofstream &out, const std::string &prefix,
                          int size) {
  for (int i = 0; i < size; i++) {
    out << "," << prefix << "_" << i;
  }
}

void write_matrix_header(std::ofstream &out, const std::string &prefix,
                          int rows, int cols) {
  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      out << "," << prefix << "_" << r << "_" << c;
    }
  }
}

void write_vector_values(std::ofstream &out, const Eigen::VectorXd &v) {
  for (int i = 0; i < v.size(); i++) {
    out << "," << v(i);
  }
}

void write_matrix_values(std::ofstream &out, const Eigen::MatrixXd &m) {
  for (int r = 0; r < m.rows(); r++) {
    for (int c = 0; c < m.cols(); c++) {
      out << "," << m(r, c);
    }
  }
}

} // namespace

FilterResult::FilterResult(const FilterConfig &filter_config)
    : filter_config_(filter_config) {};

void FilterResult::add(const FilterPredict &prediction,
                        const FilterUpdate &update) {
  predictions_.push_back(prediction);
  updates_.push_back(update);
}

void FilterResult::to_csv(const std::string &path) const {
  std::ofstream out(path);

  int state_dim = filter_config_.x_prior.size();
  int measurement_dim = updates_.empty() ? 0 : updates_[0].innovation.size();

  out << "timestep";
  write_vector_header(out, "x_predicted", state_dim);
  write_matrix_header(out, "P_predicted", state_dim, state_dim);
  write_vector_header(out, "x_updated", state_dim);
  write_vector_header(out, "innovation", measurement_dim);
  write_matrix_header(out, "S", measurement_dim, measurement_dim);
  write_matrix_header(out, "P_updated", state_dim, state_dim);
  out << "\n";

  for (std::size_t k = 0; k < predictions_.size(); k++) {
    out << k;
    write_vector_values(out, predictions_[k].x_predicted);
    write_matrix_values(out, predictions_[k].P_predicted);
    write_vector_values(out, updates_[k].x_updated);
    write_vector_values(out, updates_[k].innovation);
    write_matrix_values(out, updates_[k].S);
    write_matrix_values(out, updates_[k].P_updated);
    out << "\n";
  }
}

SmootherResult::SmootherResult(const SmootherConfig &smoother_config)
    : smoother_config_(smoother_config) {};

void SmootherResult::add(const SmootherUpdate &smoothed) {
  smoothed_.push_back(smoothed);
}

void SmootherResult::to_csv(const std::string &path) const {
  std::ofstream out(path);

  int state_dim = smoother_config_.x_prior.size();

  out << "timestep";
  write_vector_header(out, "x_smoothed", state_dim);
  write_matrix_header(out, "P_smoothed", state_dim, state_dim);
  out << "\n";

  for (std::size_t k = 0; k < smoothed_.size(); k++) {
    out << k;
    write_vector_values(out, smoothed_[k].x_smoothed);
    write_matrix_values(out, smoothed_[k].P_smoothed);
    out << "\n";
  }
}
