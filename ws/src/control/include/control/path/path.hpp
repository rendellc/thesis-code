#pragma once

#include <ignition/math/Vector2.hh>
#include <memory>
#include <vector>

class Path {
 public:
  enum class Type { LINE, CIRCLE, SPIRAL, COLLECTION };
  const Type path_type;

  Path() = delete;
  explicit Path(Type path_type) : path_type(path_type) {}
  virtual ~Path() = default;

  virtual double distance(const ignition::math::Vector2d& pos) = 0;

  virtual std::vector<ignition::math::Vector2d> sample(
      int number_of_samples) = 0;
  virtual std::vector<ignition::math::Vector2d> sample_direction(
      int number_of_samples) = 0;

  virtual ignition::math::Vector2d closest_point(
      const ignition::math::Vector2d& pos) = 0;
  virtual ignition::math::Vector2d closest_direction(
      const ignition::math::Vector2d& pos) = 0;
  virtual double closest_curvature(const ignition::math::Vector2d& pos,
                                   const ignition::math::Vector2d& vel) = 0;

  virtual ignition::math::Vector2d getBegin() const = 0;
  virtual ignition::math::Vector2d getEnd() const = 0;

  // Constructors
  static std::shared_ptr<Path> straight_line_path(
      const std::vector<ignition::math::Vector2d>& waypoints);
  static std::shared_ptr<Path> fermat_smoothing(
      const std::vector<ignition::math::Vector2d>& waypoints,
      double maximum_curvature);
  static std::shared_ptr<Path> circular_smoothing(
      const std::vector<ignition::math::Vector2d>& waypoints, double radius);
};
