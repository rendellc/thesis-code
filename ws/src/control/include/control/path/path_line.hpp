#pragma once

#include <control/path/path.hpp>

#include <ignition/math/Vector2.hh>


class PathLine : public Path
{
public:
    PathLine(const ignition::math::Vector2d& begin, const ignition::math::Vector2d& end);

    virtual ~PathLine() = default;

    double distance(const ignition::math::Vector2d& pos) override;
    
    double cross_track_error(const ignition::math::Vector2d& pos) override;

    double course_error(const ignition::math::Vector2d& pos, double course) override;

    std::vector<ignition::math::Vector2d> sample(size_t number_of_samples) override;
private:
    const ignition::math::Vector2d pos_begin, pos_end;
};


