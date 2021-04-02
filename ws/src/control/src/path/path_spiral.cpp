#include <control/path/path_spiral.hpp>


PathSpiral::PathSpiral(const ignition::math::Vector2d& origin, double orientation, double curvature, double theta_begin, double theta_end) : 
    Path(Path::Type::SPIRAL),
    origin(origin), orientation(orientation),
    curvature(curvature),
    theta_begin(theta_begin), theta_end(theta_end)
{
}

double 
PathSpiral::distance(const ignition::math::Vector2d& pos) 
{
    // TODO: nonsense
    return 0.0;
}
    
double 
PathSpiral::cross_track_error(const ignition::math::Vector2d& pos) 
{
    // TODO: nonsense
    return 0.0;
}

double 
PathSpiral::course_error(const ignition::math::Vector2d& pos, double course) 
{
    // TODO: nonsense
    return 0.0;
}

ignition::math::Vector2d
PathSpiral::spiral(double theta) const
{
    const auto sign = theta > 0 ? 1 : -1;
    return origin + sign*curvature*sqrt(fabs(theta))*ignition::math::Vector2d(
        cos(theta + orientation),
        sin(theta + orientation)
    );
}

std::vector<ignition::math::Vector2d> 
PathSpiral::sample(int number_of_samples) 
{
    std::vector<ignition::math::Vector2d> points;
    points.resize(number_of_samples);
    
    for (int i = 0; i < number_of_samples; i++)
    {
        const double theta = theta_begin + (theta_end - theta_begin) * static_cast<double>(i)/number_of_samples;
        points[i] = spiral(theta);
    }
    
    return points;
}


ignition::math::Vector2d
PathSpiral::getBegin() const
{
    return spiral(theta_begin);
}


ignition::math::Vector2d
PathSpiral::getEnd() const
{
    return spiral(theta_end);
}
