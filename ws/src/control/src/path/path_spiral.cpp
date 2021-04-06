#include <control/path/path_spiral.hpp>


PathSpiral::PathSpiral(const ignition::math::Vector2d& origin, double orientation, double scale, double theta_begin, double theta_end) : 
    Path(Path::Type::SPIRAL),
    origin(origin), orientation(orientation),
    scale(scale),
    theta_begin(theta_begin), theta_end(theta_end)
{
}

double 
PathSpiral::distance(const ignition::math::Vector2d& pos) 
{
    const double theta = find_closest_theta(pos);
    return pos.Distance(spiral(theta));
}

double
PathSpiral::distance(const ignition::math::Vector2d& pos, double theta) const
{
    return pos.Distance(spiral(theta));
}
    
double 
PathSpiral::cross_track_error(const ignition::math::Vector2d& pos) 
{
    return distance(pos);
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
    return origin + sign*scale*sqrt(fabs(theta))*ignition::math::Vector2d(
        cos(theta + orientation),
        sin(theta + orientation)
    );
}

ignition::math::Vector2d 
PathSpiral::spiral_derivative(double theta) const
{
    const auto sign = theta > 0 ? 1 : -1;
    
    return sign*scale*(
        sign/(2*sqrt(fabs(theta)))*ignition::math::Vector2d(
            cos(theta + orientation),
            sin(theta + orientation)
        ) 
        +
        sqrt(fabs(theta))*ignition::math::Vector2d(
            -sin(theta + orientation),
            cos(theta + orientation)
        )
    );
}

double
PathSpiral::distance_squared_derivative(const ignition::math::Vector2d& pos, double theta) const
{
    return 2*(spiral(theta) - pos).Dot(spiral_derivative(theta));
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

double 
PathSpiral::find_closest_theta(const ignition::math::Vector2d& pos) 
{
    const auto cache_it = closest_cache.find(pos);
    if (cache_it != closest_cache.cend())
    {
        return cache_it->second;
    }
    
    // value was not cached, compute it
    double theta = (theta_begin+theta_end)/2;
    
    constexpr int max_iterations = 100;
    constexpr double eps = 0.001;
    [[maybe_unused]]
    constexpr double stepsize = 1.0;
    auto d2_derivative = distance_squared_derivative(pos, theta);
    int iterations = 0;


    // Newtons method to find where d(distance)/dtheta = 0
    while (
        iterations < max_iterations && 
        theta_begin <= theta && theta <= theta_end && 
        fabs(d2_derivative) < eps)
    {
        [[maybe_unused]]
        const auto d = distance(pos, theta);
        // const auto d2 = d*d;
        d2_derivative = distance_squared_derivative(pos, theta);
        const auto d2_double_derivative = (
            distance_squared_derivative(pos, theta+eps) - distance_squared_derivative(pos, theta-eps)
        )/(2*eps);

        theta = theta - d2_derivative/d2_double_derivative;
        iterations++;
    }
    
    if (iterations == max_iterations)
    {
        std::cerr << "max iterations reached in Newtons method" << std::endl;
    }

    
    const auto distance_newton = distance(pos, theta);
    const auto distance_begin = distance(pos, theta_begin);
    const auto distance_end = distance(pos, theta_end);
    
    double distance_best = distance_newton;
    if (distance_begin < distance_best)
    {
        theta = theta_begin;
        distance_best = distance_begin;
    }
    if (distance_end < distance_best)
    {
        theta = theta_end;
        distance_best = distance_end;
    }

    //(*const_cast<typeof(closest_cache)*>(&closest_cache))[pos] = theta;
    closest_cache[pos] = theta;
    return theta;
}
