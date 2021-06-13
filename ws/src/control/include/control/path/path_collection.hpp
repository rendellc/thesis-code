#pragma once

#include <control/path/path.hpp>
#include <ignition/math/Vector2.hh>
#include <memory>
#include <vector>

class PathCollection : public Path {
 public:
  explicit PathCollection(const std::vector<std::shared_ptr<Path>>& collection);

  virtual ~PathCollection() = default;

  double distance(const ignition::math::Vector2d& pos) override;

  std::vector<ignition::math::Vector2d> sample(int number_of_samples) override;
  std::vector<ignition::math::Vector2d> sample_direction(
      int number_of_samples) override;

  ignition::math::Vector2d closest_point(
      const ignition::math::Vector2d& pos) override;
  ignition::math::Vector2d closest_direction(
      const ignition::math::Vector2d& pos) override;
  double closest_curvature(const ignition::math::Vector2d& pos,
                           const ignition::math::Vector2d& vel) override;
  // ignition::math::Vector2d closest_point_step_ahead(
  //     const ignition::math::Vector2d& pos, double velocity, double time,
  //     double* time_left) override;

  ignition::math::Vector2d getBegin() const override;
  ignition::math::Vector2d getEnd() const override;

 private:
  std::vector<std::shared_ptr<Path>> collection;
  int closest = 0;

  void update_closest(const ignition::math::Vector2d& pos);
};
