#pragma once

#include <ignition/math/Vector2.hh>

#include <vector>

class Path 
{
public:
    enum class Type {
        LINE, CIRCLE_ARC, SPIRAL, COLLECTION
    };
    const Type path_type;

    Path() = delete;
    Path(Type path_type) : path_type(path_type) {}
    virtual ~Path() = default;
    

    virtual double distance(const ignition::math::Vector2d& pos) = 0;
    virtual double cross_track_error(const ignition::math::Vector2d& pos) = 0;
    virtual double course_error(const ignition::math::Vector2d& pos, double course) = 0;

    virtual std::vector<ignition::math::Vector2d> sample(int number_of_samples) = 0;

    virtual ignition::math::Vector2d getBegin() const = 0;
    virtual ignition::math::Vector2d getEnd() const = 0;



    static std::shared_ptr<Path> fermat_smoothing(const std::vector<ignition::math::Vector2d>& waypoints, double maximum_curvature);
};
