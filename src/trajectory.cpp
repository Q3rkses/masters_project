#include "trajectory.hpp"
#include "utilities/math_utilities.hpp"

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
  // the whole shape so it starts at (0,0) instead, heading +x
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

Path make_figure_eight(double radius, double center_distance) {
  const double half_distance = center_distance / 2.0;
  if (half_distance <= radius) {
    throw std::invalid_argument(
        "make_figure_eight: center_distance must be more than twice the "
        "radius, otherwise the circles overlap and have no internal tangents");
  }

  // The two internal tangents of two equal circles cross at the midpoint of
  // their centers, at +-alpha to the line through the centers, and touch each
  // circle a distance tangent_length from that crossing.
  const double alpha = std::asin(radius / half_distance);
  const double tangent_length =
      std::sqrt(half_distance * half_distance - radius * radius);

  // Everything below is built with the crossing at the origin and the
  // centers on the x axis, then rotated by -alpha, so the path starts at the
  // crossing (0,0) heading +x.
  const Eigen::Rotation2Dd rotate(-alpha);
  const Eigen::Vector2d origin = Eigen::Vector2d::Zero();
  const Eigen::Vector2d up_right =
      tangent_length * Eigen::Vector2d(std::cos(alpha), std::sin(alpha));
  const Eigen::Vector2d down_right =
      tangent_length * Eigen::Vector2d(std::cos(alpha), -std::sin(alpha));
  const Eigen::Vector2d up_left = -down_right;
  const Eigen::Vector2d down_left = -up_right;

  // both turns sweep the long way round, away from the crossing
  const double long_sweep = M_PI + 2.0 * alpha;

  Path path;
  // first straight, from the crossing out to the right circle
  path.add(std::make_shared<LineSegment>(origin, rotate * up_right));
  // right circle, turning right (clockwise)
  path.add(std::make_shared<ArcSegment>(
      rotate * Eigen::Vector2d(half_distance, 0.0), radius,
      M_PI / 2.0, -long_sweep));
  // second straight, back through the crossing to the left circle
  path.add(std::make_shared<LineSegment>(rotate * down_right,
                                         rotate * up_left));
  // left circle, turning left (counterclockwise)
  path.add(std::make_shared<ArcSegment>(
      rotate * Eigen::Vector2d(-half_distance, 0.0), radius,
      M_PI / 2.0 - 2.0 * alpha, long_sweep));
  // last straight, back to the crossing where the loop closes
  path.add(std::make_shared<LineSegment>(rotate * down_left, origin));

  return path;
}

Trajectory build_trajectory(const YAML::Node &config) {
  const std::string trajectory = config["trajectory"].as<std::string>();

  if (trajectory == "rounded_rectangle") {
    return {make_rounded_rectangle(config["rectangle_width"].as<double>(),
                                   config["rectangle_height"].as<double>(),
                                   config["rectangle_corner_radius"].as<double>()),
           /*is_closed=*/true};
  }
  if (trajectory == "figure_eight") {
    return {make_figure_eight(config["figure_eight_radius"].as<double>(),
                              config["figure_eight_center_distance"].as<double>()),
           /*is_closed=*/true};
  }

  throw std::runtime_error("unknown trajectory: " + trajectory);
}

namespace {

// Quintic smoothstep 6u^5 - 15u^4 + 10u^3 on [0, 1], clamped outside it, with
// its first and second derivatives in u. Its value, slope and curvature are
// continuous at both ends, so a depth built from it has continuous vertical
// speed and acceleration.
struct Smoothstep {
  double value, slope, curvature;
};

Smoothstep smoothstep(double u) {
  if (u <= 0.0) {
    return {0.0, 0.0, 0.0};
  }
  if (u >= 1.0) {
    return {1.0, 0.0, 0.0};
  }
  const double u2 = u * u;
  return {u2 * u * (10.0 - 15.0 * u + 6.0 * u2),
          30.0 * u2 * (1.0 - 2.0 * u + u2),
          60.0 * u * (1.0 - 3.0 * u + 2.0 * u2)};
}

} // namespace

Trajectory3D::Trajectory3D(const Trajectory &planar, double speed,
                           const DiveProfile &dive, double bank_gain,
                           double roll_ramp_length, double gravity)
    : planar_(planar), speed_(speed), dive_(dive), bank_gain_(bank_gain),
      roll_ramp_length_(roll_ramp_length), gravity_(gravity) {
  const bool submerges = dive_.surface_time > dive_.dive_start;
  if (submerges) {
    if (dive_.dive_duration <= 0.0) {
      throw std::invalid_argument("dive_duration must be positive");
    }
    if (dive_.surface_time - dive_.dive_start < 2.0 * dive_.dive_duration) {
      throw std::invalid_argument(
          "the GNSS outage is too short to dive and resurface in");
    }
    // the mission depth must stay below the surface all around the loop
    double highest_y = 0.0;
    const int checks = 2000;
    for (int i = 0; i <= checks; i++) {
      const double y =
          planar_.path.sample(planar_.path.length() * i / checks).position.y();
      highest_y = std::max(highest_y, std::abs(y));
    }
    if (dive_.mission_depth - highest_y * std::abs(std::sin(dive_.tilt_angle)) <= 0.0) {
      throw std::invalid_argument(
          "mission_depth is too shallow for tilt_angle, the vehicle would "
          "break the surface during the mission");
    }
  }
}

PathSample3D Trajectory3D::sample(double t) const {
  const double path_length = planar_.path.length();

  // arc length travelled, wrapped around a closed path
  auto arc_at = [&](double arc_length) {
    if (!planar_.is_closed) {
      return arc_length;
    }
    return arc_length - path_length * std::floor(arc_length / path_length);
  };
  const double s = arc_at(speed_ * t);
  const PathSample sample = planar_.path.sample(s);

  const double c = std::cos(sample.heading);
  const double sn = std::sin(sample.heading);
  const double v = speed_;
  const double kappa = sample.curvature;

  // Depth z = h m: m is the mission depth along the loop and h the dive
  // envelope, 0 at the surface and 1 on the mission. The envelope is the
  // product of the descent ramp and the ascent ramp, so it is 1 between them
  // and 0 before the dive and after resurfacing.
  const double slope_factor = std::sin(dive_.tilt_angle);
  const double m = dive_.mission_depth + slope_factor * sample.position.y();
  const double m_rate = slope_factor * v * sn;
  const double m_accel = slope_factor * v * v * kappa * c;

  double h = 0.0, h_rate = 0.0, h_accel = 0.0;
  if (dive_.surface_time > dive_.dive_start) {
    const double T = dive_.dive_duration;
    const Smoothstep down = smoothstep((t - dive_.dive_start) / T);
    const Smoothstep up = smoothstep((dive_.surface_time - t) / T);
    h = down.value * up.value;
    h_rate = (down.slope * up.value - down.value * up.slope) / T;
    h_accel = (down.curvature * up.value - 2.0 * down.slope * up.slope +
               down.value * up.curvature) /
              (T * T);
  }

  const double z = h * m;
  const double z_rate = h_rate * m + h * m_rate;
  const double z_accel = h_accel * m + 2.0 * h_rate * m_rate + h * m_accel;

  // constant horizontal speed, so the horizontal acceleration is centripetal
  const Eigen::Vector3d position(sample.position.x(), sample.position.y(), z);
  const Eigen::Vector3d velocity(v * c, v * sn, z_rate);
  const Eigen::Vector3d acceleration(-v * v * kappa * sn, v * v * kappa * c,
                                     z_accel);

  // body x along the velocity: yaw is the path heading, pitch the flight path
  // angle, nose down (negative) while descending since z is down
  const double yaw = std::atan2(sn, c);
  const double pitch = -std::atan2(z_rate, v);
  const double yaw_rate = v * kappa;
  const double pitch_rate = -v * z_accel / (v * v + z_rate * z_rate);

  // Roll follows the curvature averaged over roll_ramp_length, which is the
  // heading change across that window over its length, so it builds up
  // smoothly where a turn starts instead of jumping.
  const double half_ramp = roll_ramp_length_ / 2.0;
  const PathSample before = planar_.path.sample(arc_at(s - half_ramp));
  const PathSample after = planar_.path.sample(arc_at(s + half_ramp));
  const double mean_curvature =
      ssa(after.heading - before.heading) / roll_ramp_length_;
  const double curvature_slope =
      (after.curvature - before.curvature) / roll_ramp_length_;
  const double bank = speed_ * speed_ * mean_curvature / gravity_;
  const double roll = bank_gain_ * std::atan(bank);
  const double roll_rate = bank_gain_ * speed_ * speed_ * speed_ *
                           curvature_slope / gravity_ / (1.0 + bank * bank);

  // body rates from the ZYX Euler rates
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  Eigen::Vector3d angular_rate;
  // clang-format off
  angular_rate << roll_rate - yaw_rate * sp,
                  pitch_rate * cr + yaw_rate * cp * sr,
                  -pitch_rate * sr + yaw_rate * cp * cr;
  // clang-format on

  const Eigen::Quaterniond attitude =
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

  return PathSample3D{
      .position = position,
      .velocity = velocity,
      .acceleration = acceleration,
      .attitude = attitude,
      .angular_rate = angular_rate,
      .euler = Eigen::Vector3d(roll, pitch, yaw),
  };
}

Trajectory3D build_trajectory_3d(const YAML::Node &config) {
  // submerged exactly while GNSS is denied, at the surface the whole run if
  // GNSS is never denied
  const DiveProfile dive{
      .dive_start = config["gnss_denied_start"].as<double>(0.0),
      .surface_time = config["gnss_denied_end"].as<double>(0.0),
      .dive_duration = config["dive_duration"].as<double>(0.0),
      .mission_depth = config["mission_depth"].as<double>(0.0),
      .tilt_angle = config["tilt_angle"].as<double>(0.0),
  };
  return Trajectory3D(build_trajectory(config), config["path_speed"].as<double>(),
                      dive, config["bank_gain"].as<double>(1.0),
                      config["roll_ramp_length"].as<double>(5.0),
                      config["gravity"].as<double>(9.81));
}
