#include <control/path/path_collection.hpp>


PathCollection::PathCollection(const std::vector<std::shared_ptr<Path>>& collection) : 
    Path(Path::Type::COLLECTION),
    collection(collection), active(0)
{
}

double 
PathCollection::distance(const ignition::math::Vector2d& pos) 
{
    // TODO: nonsense
    return 0.0;
}
    
double 
PathCollection::cross_track_error(const ignition::math::Vector2d& pos) 
{
    // TODO: nonsense
    return 0.0;
}

double 
PathCollection::course_error(const ignition::math::Vector2d& pos, double course) 
{
    // TODO: nonsense
    return 0.0;
}

std::vector<ignition::math::Vector2d> 
PathCollection::sample(int number_of_samples) 
{
    // TODO: incorrect
    std::vector<ignition::math::Vector2d> points;
    for (const auto& subpath : collection )
    {
        const auto subpath_sample = subpath->sample(number_of_samples);
        points.insert(points.end(), subpath_sample.cbegin(), subpath_sample.cend());
    }

    return points;
}


ignition::math::Vector2d
PathCollection::getBegin() const
{
    return (*collection.cbegin())->getBegin();
}


ignition::math::Vector2d
PathCollection::getEnd() const
{
    return (*collection.crbegin())->getEnd();
}
