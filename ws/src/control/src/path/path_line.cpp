#include <control/path/path_line.hpp>


PathLine::PathLine(const ignition::math::Vector2d& begin, const ignition::math::Vector2d& end) : 
    Path(Path::Type::LINE),
    pos_begin(begin), pos_end(end)
{
}

double 
PathLine::distance(const ignition::math::Vector2d& pos) 
{
    const auto direction = (pos_end-pos_begin).Normalized();
    const double t = direction.Dot(pos - pos_begin);
    if (t < 0) {
        // pos is behind begin
        return pos_begin.Distance(pos);
    } else if (t > 1) {
        // pos is ahead of end
        return pos_end.Distance(pos);
    } else {
        // pos is perpendicular to line
        const auto pos_line = pos_begin + t*direction;
        return pos_line.Distance(pos);
    }
}
    
double 
PathLine::cross_track_error(const ignition::math::Vector2d& pos) 
{
    const auto direction = (pos_end-pos_begin).Normalized();
    const double t = direction.Dot(pos - pos_begin);
    const auto pos_line = pos_begin + t*direction;
    
    return pos_line.Distance(pos);
}

double 
PathLine::course_error(const ignition::math::Vector2d& pos, double course) 
{
    const auto direction = pos_end-pos_begin;
    const auto line_course = atan2(direction.Y(), direction.X());
    return course - line_course;
}

std::vector<ignition::math::Vector2d> 
PathLine::sample(int number_of_samples) 
{
    std::vector<ignition::math::Vector2d> points;
    points.resize(number_of_samples);
    double interpolation_variable = 0.f;
    for (int i = 0; i < number_of_samples; i++)
    {
        interpolation_variable = i/static_cast<double>(number_of_samples);
        points[i] = pos_begin + interpolation_variable*(pos_end - pos_begin);
    }
    
    return points;
}