#pragma once

#include <control/path/path.hpp>
#include <ignition/math/Vector2.hh>
#include <vector>

class PathLine : public Path {
 public:
  PathLine(const ignition::math::Vector2d &begin,
           const ignition::math::Vector2d &end);

  virtual ~PathLine() = default;

  double distance(const ignition::math::Vector2d &pos) override;

  std::vector<ignition::math::Vector2d> sample(int number_of_samples) override;
  std::vector<ignition::math::Vector2d> sample_direction(
      int number_of_samples) override;

  ignition::math::Vector2d closest_point(
      const ignition::math::Vector2d &pos) override;
  ignition::math::Vector2d closest_direction(
      const ignition::math::Vector2d &pos) override;

  ignition::math::Vector2d getBegin() const override;
  ignition::math::Vector2d getEnd() const override;

 private:
  ignition::math::Vector2d pos_begin, pos_end;
};
