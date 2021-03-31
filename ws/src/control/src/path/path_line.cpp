#include <control/path/path_line.hpp>


PathLine::PathLine(const ignition::math::Vector2d& begin, const ignition::math::Vector2d& end) : 
    Path(Path::Type::LINE),
    pos_begin(begin), pos_end(end)
{
}

double 
PathLine::distance(const ignition::math::Vector2d& pos) 
{
    return 0.0;
}
    
double 
PathLine::cross_track_error(const ignition::math::Vector2d& pos) 
{
    return 0.0;
}

double 
PathLine::course_error(const ignition::math::Vector2d& pos, double course) 
{
    return 0.0;
}

std::vector<ignition::math::Vector2d> 
PathLine::sample(size_t number_of_samples) 
{
    std::vector<ignition::math::Vector2d> points;
    points.resize(number_of_samples);
    double interpolation_variable = 0.f;
    for (size_t i = 0; i < number_of_samples; i++)
    {
        interpolation_variable = i/static_cast<double>(number_of_samples);
        points[i] = pos_begin + interpolation_variable*(pos_end - pos_begin);
    }
    
    return points;
}