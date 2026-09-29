/**
 * @file trajectory.hpp
 * @brief Ground-truth paths for exercising the 2D INS model: a path is a
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
 * @brief A circle of the given radius, centered at the origin, traced
 * counterclockwise starting at angle 0.
 */
Path make_circle(double radius);

/**
 * @brief A straight run, followed by a turn.
 * @param straight_length length of the initial straight, starting at the
 * origin heading along +x
 * @param turn_radius radius of the turn that follows
 * @param turn_sweep signed angle swept by the turn, in radians; positive
 * turns left, negative turns right
 */
Path make_straight_into_turn(double straight_length, double turn_radius,
                             double turn_sweep);

#endif
