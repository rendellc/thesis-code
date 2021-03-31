// #include <control/path/path.hpp>
#include <control/path/path_line.hpp>

#include <ignition/math/Vector2.hh>
#include <memory>

using Vec2 = ignition::math::Vector2d;

int main()
{
    std::shared_ptr<Path> path = std::make_shared<PathLine>(
        Vec2(0,0), Vec2(10,20)
    );
    
    const auto points = path->sample(100);
    for (const auto& p : points)
    {
        std::cout << p << std::endl;
    }
    
    return 0;
}