#pragma once

#include <control/path/path.hpp>
#include <control/path/path_line.hpp>

#include <ignition/math/Vector2.hh>


class PathSpiral : public Path
{
public:
    PathSpiral(const ignition::math::Vector2d& begin, const ignition::math::Vector2d& mid, const ignition::math::Vector2d& end, double maximum_curvature);

    virtual ~PathSpiral() = default;

    double distance(const ignition::math::Vector2d& pos) override;
    
    double cross_track_error(const ignition::math::Vector2d& pos) override;

    double course_error(const ignition::math::Vector2d& pos, double course) override;

    std::vector<ignition::math::Vector2d> sample(int number_of_samples) override;
    
private:
    const ignition::math::Vector2d pos_begin, pos_mid, pos_end;
    const double maximum_curvature;
    
    // Spiral parameters
    ignition::math::Vector2d spiral_begin, spiral_end;
    int turn_direction;
    double course_in, course_out, course_change;
    double theta_end;
    double theta_max_curvature;
    double curvature;
    double alpha;
    double h, l1, l2, l;

    // line before and after spiral
    std::shared_ptr<PathLine> line_before, line_after;
};
