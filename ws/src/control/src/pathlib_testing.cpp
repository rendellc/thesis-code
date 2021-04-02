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
        Vec2(7,3),
        Vec2(10,10),
        Vec2(5,15),
        Vec2(0,10)
    };
    
    //PathSpiral(const ignition::math::Vector2d& origin, double orientation, double curvature, double theta_begin, double theta_end);
    // std::shared_ptr<Path> path = std::make_shared<PathSpiral>(
    //     Vec2(5,5), 1.57/2, 1, -1, 1
    // );
    // 
    // const auto points = path->sample(100);
    // for (const auto& p : points)
    // {
    //     std::cout << p.X() << "," << p.Y() << "\n";
    // }
    
    
// #if 0
    std::vector<std::shared_ptr<Path>> paths;

    constexpr double minimum_turn_radius = 0.5;
    constexpr double maximum_curvature = 1/minimum_turn_radius;
    const double PI = atan2(+0.0,-1.0);
    const auto ssa = [](double angle){ return atan2(sin(angle), cos(angle)); };
    for (int i = 1; i < waypoints.size() - 1; i++)
    {
        const auto prev = (paths.empty()) ? waypoints[i-1] : (*paths.crbegin())->getEnd();
        const auto& curr = waypoints[i];

        const auto& next = waypoints[i+1];

        const auto direction_in = (curr - prev).Normalized();
        const auto direction_out = (next - curr).Normalized();
        const double course_in = atan2(direction_in.Y(), direction_in.X());
        const double course_out = atan2(direction_out.Y(), direction_out.X());
        const double course_change = fabs(ssa(course_out - course_in));
        const int turn_direction = ssa(course_out - course_in) > 0 ? 1 : -1;
        
        // std::cout << "course change " << course_change << " " << turn_direction << std::endl;
        
        // Halleys method to determine theta_end
        double theta_end = 0.0;
        constexpr double threshold = 0.001;
        const auto f = [=](double theta_end){
            // f(theta_end) = 0 defines theta_end
            return theta_end + atan(2*theta_end) - course_change/2;
        };
        const auto df = [=](double theta_end){
            return 1.0 + 2/(1+4*pow(theta_end,2));
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
        }
        
        const double theta_max_curvature = std::min(
            theta_end, 
            sqrt(sqrt(7)/2 - 1.25)
        );
        const double curvature = 1/maximum_curvature * 
            2*sqrt(theta_max_curvature) * (3 + 4*pow(theta_max_curvature,2))
            /
            sqrt(pow(1+4*pow(theta_max_curvature,2),3));
    
        // std::cout << "theta " << theta_end << " " << theta_max_curvature << std::endl;
        
        const double alpha = (PI - course_change)/2;
        const double h = curvature*sqrt(theta_end)*sin(theta_end);
        const double l1 = curvature*sqrt(theta_end)*cos(theta_end);
        const double l2 = h/tan(alpha);
        const double l = l1 + l2;



        const auto spiral_begin = curr - l*direction_in;
        const auto spiral_end = curr + l*direction_out;

        paths.push_back(
            std::make_shared<PathLine>(
                prev, spiral_begin
            )
        );
        paths.push_back(
            std::make_shared<PathSpiral>(
                spiral_begin, course_in, curvature, 0, theta_end
            )
        );
        paths.push_back(
            std::make_shared<PathSpiral>(
                spiral_end, course_out, curvature, -theta_end, 0
            )
        );
        
        // std::cerr << "spiral 1 end:" << s1->getEnd() << "\n";
        // std::cerr << "spiral 2 begin:" << s2->getBegin();
    }
    
    const auto secondtolast = (paths.empty()) ? waypoints[0] : (*paths.crbegin())->getEnd();
    paths.push_back(
        std::make_shared<PathLine>(
            secondtolast, *waypoints.crbegin()
        )
    );
    
    for (const auto& path : paths)
    {
        const auto points = path->sample(50);
        for (const auto& p : points)
        {
            std::cout << p.X() << "," << p.Y() << "\n";
        }
    }

    
    // constexpr double min_turn_radius = 1.0;

    // auto path = std::make_shared<PathSpiral>(
    //      Vec2(0,0), 0, 1/min_turn_radius, 1, 0.5
    // );
    // //waypoints[0], waypoints[1], waypoints[2], 1/min_turn_radius
    // 

    //Vec2 testpoint = Vec2(0,-1);
    //std::cout << path->distance(Vec2(0,-1)) << std::endl;
    //std::cout << path->distance(Vec2(0,0)) << std::endl;
    //std::cout << path->distance(Vec2(0,0.5)) << std::endl;
    //std::cout << path->distance(Vec2(0,1)) << std::endl;
    //std::cout << path->distance(Vec2(0,10)) << std::endl;
    //std::cout << path->distance(Vec2(0,15)) << std::endl;
// #endif
    
    return 0;
}
