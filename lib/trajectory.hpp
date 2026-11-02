/**
 * @file trajectory.hpp
 * @brief Ground-truth paths for exercising the 2D and 3D INS models: a path is a
 * sequence of lines and circular arcs, each with an exact closed-form
 * length and an exact closed-form sample as a function of arc length. No
 * fitting, lookup tables or numerical differentiation are involved, since
 * lines and arcs already have exact constant-speed parametrizations.
 */

#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <yaml-cpp/yaml.h>

/**
 * @brief One sample of a path at a given arc length: where it is, which way
 * it points, and how sharply it is turning there.
 */
struct PathSample {
  Eigen::Vector2d position;
  double heading;   // tangent direction, radians
  double curvature; // signed, positive = turning left (counterclockwise)
};

/**
 * @brief One geometric piece of a path. Each kind knows its own length and
 * can be sampled exactly, in closed form, as a function of arc length
 * measured from its own start.
 */
class PathSegment {
public:
  virtual ~PathSegment() = default;

  /**
   * @brief The length of this segment.
   */
  virtual double length() const = 0;

  /**
   * @brief Samples this segment.
   * @param arc_length distance travelled from this segment's own start,
   * expected to be in [0, length()]
   */
  virtual PathSample sample(double arc_length) const = 0;
};

/**
 * @brief A straight line from start to end, at constant heading and zero
 * curvature.
 */
class LineSegment final : public PathSegment {
public:
  LineSegment(const Eigen::Vector2d &start, const Eigen::Vector2d &end);

  double length() const override;
  PathSample sample(double arc_length) const override;

private:
  Eigen::Vector2d start_;
  Eigen::Vector2d direction_; // unit vector, start to end
  double length_;
};

/**
 * @brief A circular arc, at constant curvature 1/radius.
 */
class ArcSegment final : public PathSegment {
public:
  /**
   * @param center center of the circle
   * @param radius radius, must be positive
   * @param start_angle angle in radians, relative to center, of the arc's
   * start point
   * @param sweep signed angle swept from start_angle, in radians; positive
   * turns left (counterclockwise), negative turns right
   */
  ArcSegment(const Eigen::Vector2d &center, double radius, double start_angle,
             double sweep);

  double length() const override;
  PathSample sample(double arc_length) const override;

private:
  Eigen::Vector2d center_;
  double radius_;
  double start_angle_;
  double sweep_;
};

/**
 * @brief A path made of segments joined end to end, sampled by total arc
 * length travelled from the path's start. Segments are expected to already
 * join up positionally and in heading; Path does not check or enforce this.
 */
class Path {
public:
  void add(std::shared_ptr<PathSegment> segment);

  /**
   * @brief The path's total length, i.e. the sum of its segments' lengths.
   */
  double length() const;

  /**
   * @brief Samples the path at the given arc length from its start, which
   * is clamped into [0, length()], so a path is stationary at its last
   * sample once arc_length exceeds it.
   */
  PathSample sample(double arc_length) const;

private:
  std::vector<std::shared_ptr<PathSegment>> segments_;
};

/**
 * @brief A rounded rectangle: four straight sides joined by quarter-circle
 * fillets at the corners, traced counterclockwise starting at the middle of
 * the first (bottom) side. The classic "four corner test" for exercising an
 * INS's turn response and bias observability.
 * @param width, height the rectangle's outer size, centered at the origin
 * @param corner_radius fillet radius at each corner; must be less than half
 * of both width and height
 */
Path make_rounded_rectangle(double width, double height, double corner_radius);

/**
 * @brief A figure-eight: two equal circles joined by two straights that
 * cross at the origin (the circles' internal tangents), traced starting at
 * the crossing heading +x. It turns right around one circle and left around
 * the other, with a straight between each turn, so a lap exercises both turn
 * directions and goes through the same point twice.
 * @param radius radius of both circles
 * @param center_distance distance between the two circle centers, must be
 * more than twice the radius
 */
Path make_figure_eight(double radius, double center_distance);

/**
 * @brief A path together with how it is traversed.
 */
struct Trajectory {
  Path path;
  bool is_closed; // loop around the path instead of stopping at its end
};

/**
 * @brief Picks the trajectory named by the "trajectory" key and builds it
 * from its own set of keys; only the keys of the picked shape need to be
 * present in the config
 * @param config the parsed yaml config
 * @return the path and whether it is closed
 */
Trajectory build_trajectory(const YAML::Node &config);


// The 3D paths are the shapes above laid in a plane tilted about the x axis, so
// the depth changes smoothly along the loop. Frames follow the 3D scenario:
// navigation frame NED (z down), attitude q rotates body to navigation, ZYX
// Euler angles (roll phi, pitch theta, yaw psi), body x along the velocity.

/**
 * @brief The true motion at one instant. Gravity is not in here: the IMU
 * reads the specific force R^T (acceleration - g), with g = [0, 0, gravity]
 * in NED, which the caller forms from these fields.
 */
struct PathSample3D {
  Eigen::Vector3d position;         // navigation frame [m]
  Eigen::Vector3d velocity;         // navigation frame [m/s]
  Eigen::Vector3d acceleration;     // navigation frame, kinematic [m/s^2]
  Eigen::Quaterniond attitude;      // body to navigation, w x y z, unit
  Eigen::Vector3d angular_rate;     // body frame, [p, q, r] [rad/s]
  Eigen::Vector3d euler;            // [roll, pitch, yaw] [rad]
};

/**
 * @brief A 2D path lifted into a tilted plane and traversed at constant speed.
 */
class Trajectory3D {
public:
  /**
   * @param planar the 2D path and whether it loops
   * @param speed constant speed along the path [m/s]
   * @param tilt_angle rotation of the path plane about the x axis [rad]; the
   * depth is then z = y sin(tilt_angle)
   * @param bank_gain share of the coordinated-turn roll applied in turns,
   * 0 keeps the vehicle level, 1 banks by atan(speed^2 curvature / gravity)
   * @param roll_ramp_length arc length over which the roll builds up when a
   * turn starts or ends [m], so the roll is continuous
   * @param gravity magnitude of gravity [m/s^2], only used for the bank angle
   */
  Trajectory3D(const Trajectory &planar, double speed, double tilt_angle,
               double bank_gain, double roll_ramp_length, double gravity);

  /**
   * @brief The true motion at time t from the start, looping when the path
   * is closed and stopping at its end when it is not.
   */
  PathSample3D sample(double t) const;

private:
  Trajectory planar_;
  double speed_;
  double tilt_angle_;
  double bank_gain_;
  double roll_ramp_length_;
  double gravity_;
};

/**
 * @brief Builds the trajectory named by the "trajectory" key
 * (rounded_rectangle | figure_eight) with the same shape keys as the 2D
 * build_trajectory, plus
 *   path_speed        constant speed [m/s]
 *   tilt_angle        tilt of the path plane about the x axis [rad]
 *   bank_gain         0 to 1, share of the coordinated-turn roll (default 1)
 *   roll_ramp_length  arc length the roll takes to build up [m] (default 5)
 *   gravity           [m/s^2] (default 9.81)
 */
Trajectory3D build_trajectory_3d(const YAML::Node &config);

#endif
