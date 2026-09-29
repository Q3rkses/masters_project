#include "trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

LineSegment::LineSegment(const Eigen::Vector2d &start,
                         const Eigen::Vector2d &end)
    : start_(start), direction_((end - start).normalized()),
      length_((end - start).norm()) {}

double LineSegment::length() const { return length_; }

PathSample LineSegment::sample(double arc_length) const {
  return PathSample{
      .position = start_ + arc_length * direction_,
      .heading = std::atan2(direction_.y(), direction_.x()),
      .curvature = 0.0,
  };
}

ArcSegment::ArcSegment(const Eigen::Vector2d &center, double radius,
                       double start_angle, double sweep)
    : center_(center), radius_(radius), start_angle_(start_angle),
      sweep_(sweep) {}

double ArcSegment::length() const { return radius_ * std::abs(sweep_); }

PathSample ArcSegment::sample(double arc_length) const {
  const double direction = sweep_ >= 0.0 ? 1.0 : -1.0;
  const double angle = start_angle_ + direction * arc_length / radius_;

  return PathSample{
      .position =
          center_ + radius_ * Eigen::Vector2d(std::cos(angle), std::sin(angle)),
      .heading = angle + direction * (M_PI / 2.0),
      .curvature = direction / radius_,
  };
}

void Path::add(std::shared_ptr<PathSegment> segment) {
  segments_.push_back(std::move(segment));
}

double Path::length() const {
  double total = 0.0;
  for (const std::shared_ptr<PathSegment> &segment : segments_) {
    total += segment->length();
  }
  return total;
}

PathSample Path::sample(double arc_length) const {
  if (segments_.empty()) {
    throw std::runtime_error("Path::sample: path has no segments");
  }

  arc_length = std::clamp(arc_length, 0.0, length());

  // walk the segments until arc_length falls within one of them. paths only
  // ever have a handful of segments, so a linear scan is simplest.
  double offset = 0.0;
  for (size_t i = 0; i < segments_.size(); i++) {
    const double segment_length = segments_[i]->length();
    const bool last_segment = (i + 1 == segments_.size());
    if (arc_length <= offset + segment_length || last_segment) {
      return segments_[i]->sample(arc_length - offset);
    }
    offset += segment_length;
  }

  throw std::runtime_error("Path::sample: unreachable");
}

Path make_rounded_rectangle(double width, double height,
                            double corner_radius) {
  if (corner_radius >= width / 2.0 || corner_radius >= height / 2.0) {
    throw std::invalid_argument(
        "make_rounded_rectangle: corner_radius must be less than half of "
        "both width and height");
  }

  const double half_width = width / 2.0;
  const double half_height = height / 2.0;
  const double r = corner_radius;
  const double quarter_turn = M_PI / 2.0;

  // every point below is for a rectangle centered at the origin; this shifts
  // the whole shape so it starts at (0,0) instead, heading +x, like the
  // other two trajectories
  const Eigen::Vector2d shift(half_width - r, half_height);

  Path path;

  // bottom side, left to right
  path.add(std::make_shared<LineSegment>(
      shift + Eigen::Vector2d(-half_width + r, -half_height),
      shift + Eigen::Vector2d(half_width - r, -half_height)));
  // bottom-right corner
  path.add(std::make_shared<ArcSegment>(
      shift + Eigen::Vector2d(half_width - r, -half_height + r), r,
      -quarter_turn, quarter_turn));
  // right side, bottom to top
  path.add(std::make_shared<LineSegment>(
      shift + Eigen::Vector2d(half_width, -half_height + r),
      shift + Eigen::Vector2d(half_width, half_height - r)));
  // top-right corner
  path.add(std::make_shared<ArcSegment>(
      shift + Eigen::Vector2d(half_width - r, half_height - r), r, 0.0,
      quarter_turn));
  // top side, right to left
  path.add(std::make_shared<LineSegment>(
      shift + Eigen::Vector2d(half_width - r, half_height),
      shift + Eigen::Vector2d(-half_width + r, half_height)));
  // top-left corner
  path.add(std::make_shared<ArcSegment>(
      shift + Eigen::Vector2d(-half_width + r, half_height - r), r,
      quarter_turn, quarter_turn));
  // left side, top to bottom
  path.add(std::make_shared<LineSegment>(
      shift + Eigen::Vector2d(-half_width, half_height - r),
      shift + Eigen::Vector2d(-half_width, -half_height + r)));
  // bottom-left corner, closes the loop back onto the bottom side's start
  path.add(std::make_shared<ArcSegment>(
      shift + Eigen::Vector2d(-half_width + r, -half_height + r), r, M_PI,
      quarter_turn));

  return path;
}

Path make_circle(double radius) {
  // center placed so the circle starts at (0,0) heading +x (tangent to the
  // circle at its bottom point), like the other two trajectories
  Path path;
  path.add(std::make_shared<ArcSegment>(Eigen::Vector2d(0.0, radius), radius,
                                        -M_PI / 2.0, 2.0 * M_PI));
  return path;
}

Path make_straight_into_turn(double straight_length, double turn_radius,
                             double turn_sweep) {
  const Eigen::Vector2d start = Eigen::Vector2d::Zero();
  const Eigen::Vector2d end(straight_length, 0.0);

  Path path;
  path.add(std::make_shared<LineSegment>(start, end));

  // the turn's center sits a radius to the left (or right) of where the
  // straight ends, perpendicular to its heading of 0
  const double side = turn_sweep >= 0.0 ? 1.0 : -1.0;
  const Eigen::Vector2d center = end + turn_radius * Eigen::Vector2d(0.0, side);
  const Eigen::Vector2d center_to_end = end - center;
  const double start_angle =
      std::atan2(center_to_end.y(), center_to_end.x());

  path.add(std::make_shared<ArcSegment>(center, turn_radius, start_angle,
                                        turn_sweep));
  return path;
}
