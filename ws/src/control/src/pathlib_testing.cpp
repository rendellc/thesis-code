// #include <control/path/path.hpp>
#include <control/path/path_line.hpp>
#include <control/path/path_spiral.hpp>

#include <ignition/math/Vector2.hh>
#include <memory>

using Vec2 = ignition::math::Vector2d;

int main()
{
    std::vector<Vec2> waypoints = {
        Vec2(0,0),
        Vec2(10,0),
        Vec2(10,10),
        Vec2(5,15),
        Vec2(0,10)
    };
    
    constexpr double min_turn_radius = 1.0;

    auto path = std::make_shared<PathSpiral>(
        waypoints[0], waypoints[1], waypoints[2], 1/min_turn_radius
    );
    
    const auto points = path->sample(50);
    for (const auto& p : points)
    {
        std::cout << p << std::endl;
    }

    //Vec2 testpoint = Vec2(0,-1);
    //std::cout << path->distance(Vec2(0,-1)) << std::endl;
    //std::cout << path->distance(Vec2(0,0)) << std::endl;
    //std::cout << path->distance(Vec2(0,0.5)) << std::endl;
    //std::cout << path->distance(Vec2(0,1)) << std::endl;
    //std::cout << path->distance(Vec2(0,10)) << std::endl;
    //std::cout << path->distance(Vec2(0,15)) << std::endl;
    
    
    return 0;
}