#include <control/path/path_spiral.hpp>


PathSpiral::PathSpiral(const ignition::math::Vector2d& begin, const ignition::math::Vector2d& mid, const ignition::math::Vector2d& end, double maximum_curvature) : 
    Path(Path::Type::SPIRAL),
    pos_begin(begin), pos_mid(mid), pos_end(end),
    maximum_curvature(maximum_curvature)
{
    const ignition::math::Vector2d direction_in = (pos_mid - pos_begin).Normalized();
    const ignition::math::Vector2d direction_out = (pos_end - pos_mid).Normalized();
    
    course_in = atan2(direction_in.Y(), direction_in.X());
    course_out = atan2(direction_out.Y(), direction_out.X());
    course_change = fabs(course_out - course_in);
    if (course_out > course_in)
    {
        turn_direction = 1;
    } else 
    {
        turn_direction = -1;
    }
    
    // Halley's method to determine theta_end
    int iterations = 0;
    theta_end = 0.0;
    constexpr double threshold = 0.0001;
    const auto f = [=](double theta_end){
        // f(theta_end) = 0 defines theta_end
        return theta_end + atan(2*theta_end) - course_change/2;
    };
    const auto df = [=](double theta_end){
        return 1.0 + 2/(1 + 4*pow(theta_end,2));
    };
    const auto ddf = [=](double theta_end){
        return -16*theta_end/pow(1 + 4*pow(theta_end,2), 2);
    };
    while (fabs(f(theta_end)) > threshold)
    {
        const double y = f(theta_end);
        const double dy = df(theta_end);
        const double ddy = ddf(theta_end);
        
        theta_end = theta_end - 2*y*dy/(2*pow(dy,2) - y*ddy);
        iterations++;
    }
    
    const double spiral_max_curvature_theta = sqrt(7)/2 - 1.25;
    if (theta_end < spiral_max_curvature_theta) 
    {
        theta_max_curvature = theta_end;
    } else
    {
        theta_max_curvature = spiral_max_curvature_theta;
    }
    curvature = 1/maximum_curvature * 
        2*sqrt(theta_max_curvature) * (3 + 4*pow(theta_max_curvature,2))
        /
        pow(sqrt(1 + 4*pow(theta_max_curvature,2)),3);
    
    const double PI = atan2(+0.0,-1.0);
    
    alpha = (PI - course_change)/2;
    h = curvature*sqrt(theta_end)*sin(theta_end);
    l1 = curvature*sqrt(theta_end)*cos(theta_end);
    l2 = h/tan(alpha);
    l = l1 + l2;

    spiral_begin = pos_mid - l*direction_in;
    spiral_end = pos_mid + l*direction_out;

    line_before = std::make_shared<PathLine>(pos_begin, spiral_begin);
    line_after = std::make_shared<PathLine>(spiral_end, pos_end);
}

double 
PathSpiral::distance(const ignition::math::Vector2d& pos) 
{
    // TODO: nonsense
    return pos.Distance(ignition::math::Vector2d(0,0));
}
    
double 
PathSpiral::cross_track_error(const ignition::math::Vector2d& pos) 
{
    // TODO: nonsense
    return distance(pos);
}

double 
PathSpiral::course_error(const ignition::math::Vector2d& pos, double course) 
{
    // TODO: nonsense
    return distance(pos) + course;
}

std::vector<ignition::math::Vector2d> 
PathSpiral::sample(int number_of_samples) 
{
    std::vector<ignition::math::Vector2d> points;
    points.resize(number_of_samples);
    
    auto point_iterator = points.begin();
    
    // line before
    auto subpoints = line_before->sample(number_of_samples/4);
    point_iterator = std::copy(subpoints.cbegin(), subpoints.cend(), point_iterator);
    
    // first spiral
    for (int i = 0; i < number_of_samples/4; i++)
    {
        const double theta = 4*static_cast<double>(i)/number_of_samples;
        subpoints[i] = spiral_begin + curvature*sqrt(theta)*ignition::math::Vector2d(
            cos(turn_direction*theta + course_in),
            sin(turn_direction*theta + course_in)
        );
    }
    point_iterator = std::copy(subpoints.cbegin(), subpoints.cend(), point_iterator);

    // mirrored spiral
    for (int i = number_of_samples/4 - 1; i >= 0; i--)
    {
        const double theta = 4*static_cast<double>(i)/number_of_samples;
        subpoints[i] = spiral_end - curvature*sqrt(theta)*ignition::math::Vector2d(
            cos(-turn_direction*theta + course_out),
            sin(-turn_direction*theta + course_out)
        );
    }
    point_iterator = std::copy(subpoints.cbegin(), subpoints.cend(), point_iterator);

    // line after
    subpoints = line_after->sample(number_of_samples - 3*(number_of_samples/4));
    point_iterator = std::copy(subpoints.cbegin(), subpoints.cend(), point_iterator);
    
    if (point_iterator != points.end())
    {
        std::cerr << "Warning: point_iterator not aligned with end of points array" << std::endl;
    }
    
    return points;
}