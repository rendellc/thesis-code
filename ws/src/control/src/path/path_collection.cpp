#include <control/path/path_collection.hpp>


PathCollection::PathCollection(const std::vector<std::shared_ptr<Path>>& collection) : 
    Path(Path::Type::COLLECTION),
    collection(collection), closest(0)
{
}

double 
PathCollection::distance(const ignition::math::Vector2d& pos) 
{
    update_closest(pos);
    return collection[closest]->distance(pos);
}
    
double 
PathCollection::cross_track_error(const ignition::math::Vector2d& pos) 
{
    update_closest(pos);
    return collection[closest]->cross_track_error(pos);
}

double 
PathCollection::course_error(const ignition::math::Vector2d& pos, double course) 
{
    update_closest(pos);
    return collection[closest]->course_error(pos, course);
}

std::vector<ignition::math::Vector2d> 
PathCollection::sample(int number_of_samples) 
{
    // Bug: might make fewer samples due to integere division

    const int number_of_samples_per = number_of_samples/collection.size();
    std::vector<ignition::math::Vector2d> points;
    for (const auto& subpath : collection)
    {
        const auto subpath_sample = subpath->sample(number_of_samples_per);
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


void 
PathCollection::update_closest(const ignition::math::Vector2d& pos)
{
    closest = 0;
    double smallest_distance = collection[0]->distance(pos);
    for (int i = 1; i < collection.size(); i++)
    {
        const double distance = collection[i]->distance(pos);
        if (distance < smallest_distance)
        {
            closest = i;
            smallest_distance = distance;
        }
    }
}