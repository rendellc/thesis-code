#include <control/path/path_collection.hpp>

PathCollection::PathCollection(
    const std::vector<std::shared_ptr<Path>>& collection)
    : Path(Path::Type::COLLECTION), collection(collection), closest(0) {}

double PathCollection::distance(const ignition::math::Vector2d& pos) {
  update_closest(pos);
  return collection[closest]->distance(pos);
}

std::vector<ignition::math::Vector2d> PathCollection::sample(
    int number_of_samples) {
  // Bug: might make fewer samples due to integer division
  const int number_of_samples_per = number_of_samples / collection.size();
  // std::cout << "Subpaths (" << collection.size()
  //           << ") with number_of_samples_per = " << number_of_samples_per
  //           << std::endl;
  std::vector<ignition::math::Vector2d> points;
  for (const auto& subpath : collection) {
    const auto subpath_sample = subpath->sample(number_of_samples_per);
    points.insert(points.end(), subpath_sample.cbegin(), subpath_sample.cend());
  }

  return points;
}

std::vector<ignition::math::Vector2d> PathCollection::sample_direction(
    int number_of_samples) {
  // Bug: might make fewer samples due to integer division
  const int number_of_samples_per = number_of_samples / collection.size();
  std::vector<ignition::math::Vector2d> points;
  for (const auto& subpath : collection) {
    const auto subpath_sample =
        subpath->sample_direction(number_of_samples_per);
    points.insert(points.end(), subpath_sample.cbegin(), subpath_sample.cend());
  }

  return points;
}

ignition::math::Vector2d PathCollection::closest_point(
    const ignition::math::Vector2d& pos) {
  update_closest(pos);
  return collection[closest]->closest_point(pos);
}

ignition::math::Vector2d PathCollection::closest_direction(
    const ignition::math::Vector2d& pos) {
  update_closest(pos);
  return collection[closest]->closest_direction(pos);
}

double PathCollection::closest_curvature(const ignition::math::Vector2d& pos,
                                         const ignition::math::Vector2d& vel) {
  update_closest(pos);
  return collection[closest]->closest_curvature(pos, vel);
}

// ignition::math::Vector2d PathCollection::closest_point_step_ahead(
//     const ignition::math::Vector2d& pos, double velocity, double time,
//     double* time_left_p) {
//   update_closest(pos);
//
//   double time_left = time;
//   if (time_left_p) {
//     time_left = *time_left_p;
//   }
//
//   int subpath_index = closest;
//   ignition::math::Vector2d step_ahead_pos;
//   while (time_left > 0 && subpath_index < collection.size()) {
//     step_ahead_pos = collection[subpath_index]->closest_point_step_ahead(
//         pos, velocity, time_left, time_left_p);
//
//     subpath_index++;
//   }
//
//   return step_ahead_pos;
// }

ignition::math::Vector2d PathCollection::getBegin() const {
  return (*collection.cbegin())->getBegin();
}

ignition::math::Vector2d PathCollection::getEnd() const {
  return (*collection.crbegin())->getEnd();
}

void PathCollection::update_closest(const ignition::math::Vector2d& pos) {
  double smallest_distance = std::numeric_limits<double>::infinity();
  for (int i = 0; i < collection.size(); i++) {
    const double distance = collection[i]->distance(pos);
    if (distance < smallest_distance) {
      closest = i;
      smallest_distance = distance;
    }
  }
}
