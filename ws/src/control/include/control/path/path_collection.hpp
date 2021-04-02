#pragma once

#include <control/path/path.hpp>

#include <ignition/math/Vector2.hh>


class PathCollection : public Path
{
public:
    PathCollection(const std::vector<std::shared_ptr<Path>>& collection);

    virtual ~PathCollection() = default;

    double distance(const ignition::math::Vector2d& pos) override;
    
    double cross_track_error(const ignition::math::Vector2d& pos) override;

    double course_error(const ignition::math::Vector2d& pos, double course) override;

    std::vector<ignition::math::Vector2d> sample(int number_of_samples) override;
    
    ignition::math::Vector2d getBegin() const override;
    ignition::math::Vector2d getEnd() const override;


private:
    std::vector<std::shared_ptr<Path>> collection;
    int active = 0;
};
