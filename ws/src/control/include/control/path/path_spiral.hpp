#pragma once

#include <control/path/path.hpp>
#include <control/path/path_line.hpp>

#include <ignition/math/Vector2.hh>


class PathSpiral : public Path
{
public:
    PathSpiral(const ignition::math::Vector2d& origin, double orientation, double scale, double theta_begin, double theta_end);

    virtual ~PathSpiral() = default;

    double distance(const ignition::math::Vector2d& pos) override;
    
    double cross_track_error(const ignition::math::Vector2d& pos) override;

    double course_error(const ignition::math::Vector2d& pos, double course) override;

    std::vector<ignition::math::Vector2d> sample(int number_of_samples) override;
    
    ignition::math::Vector2d getBegin() const override;
    ignition::math::Vector2d getEnd() const override;
    
private:
    const ignition::math::Vector2d origin;
    const double orientation;
    const double scale;
    const double theta_begin, theta_end;
    

    
    ignition::math::Vector2d spiral(double theta) const;
    ignition::math::Vector2d spiral_derivative(double theta) const;
    

    std::map<ignition::math::Vector2d, double> closest_cache;
    double find_closest_theta(const ignition::math::Vector2d& pos);
    
    double distance(const ignition::math::Vector2d& pos, double theta) const;
    double distance_squared_derivative(const ignition::math::Vector2d& pos, double theta) const;
    

    


    // const double course_in;
    // const double scale;
    // const int turn_direction;
    // const double theta_end;
    // const bool reversed = false;
    
    // Spiral parameters
    // ignition::math::Vector2d spiral_begin, spiral_end;
    // int turn_direction;
    // double course_in, course_out, course_change;
    // double theta_end;
    // double theta_max_curvature;
    // double scale;
    // double alpha;
    // double h, l1, l2, l;

    // // line before and after spiral
    // std::shared_ptr<PathLine> line_before, line_after;
};
