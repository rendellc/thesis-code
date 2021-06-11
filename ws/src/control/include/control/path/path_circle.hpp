#pragma once

#include <control/path/path.hpp>
#include <ignition/math/Vector2.hh>
#include <vector>

class PathCircle : public Path {
 public:
  PathCircle(const ignition::math::Vector2d &center, double radius,
             double angle_begin, double angle_end);

  virtual ~PathCircle() = default;

  double distance(const ignition::math::Vector2d &pos) override;

  std::vector<ignition::math::Vector2d> sample(int number_of_samples) override;
  std::vector<ignition::math::Vector2d> sample_direction(
      int number_of_samples) override;

  double closest_angle(const ignition::math::Vector2d &pos);
  ignition::math::Vector2d closest_point(
      const ignition::math::Vector2d &pos) override;
  ignition::math::Vector2d closest_direction(
      const ignition::math::Vector2d &pos) override;
  double closest_courserate(const ignition::math::Vector2d &pos,
                            const ignition::math::Vector2d &vel) override;
  // ignition::math::Vector2d closest_point_step_ahead(
  //     const ignition::math::Vector2d &pos, double velocity, double time,
  //     double *time_left) override;

  ignition::math::Vector2d getBegin() const override;
  ignition::math::Vector2d getEnd() const override;

  ignition::math::Vector2d circle_position(double theta) const;
  ignition::math::Vector2d circle_velocity(double theta) const;

 private:
  ignition::math::Vector2d center;
  double radius;
  double angle_begin, angle_end;
  double domain_lower, domain_upper;
};
