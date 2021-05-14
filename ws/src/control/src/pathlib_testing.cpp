// #include <control/path/path.hpp>
#include <control/path/path_line.hpp>
#include <control/path/path_spiral.hpp>
#include <ignition/math/Vector2.hh>
#include <memory>
#include <string>

using Vec2 = ignition::math::Vector2d;
using std::string;
using std::vector;

vector<string> split(const string& str, char split_char) {
  std::stringstream ss(str);
  vector<string> result;
  while (ss.good()) {
    string substr;
    getline(ss, substr, split_char);
    result.push_back(substr);
  }

  return result;
}

Vec2 to_vec2(const std::string& str) {
  const auto xy = split(str, ',');
  return Vec2(std::stod(xy[0]), std::stod(xy[1]));
}

int main(int argc, char* argv[]) {
  std::vector<std::string> arguments;
  for (int i = 1; i < argc; i++) {
    arguments.push_back(argv[i]);
  }

  double radius = -1.0;
  double maximum_curvature = -1.0;
  int num_samples = 0;
  std::string smoothing;
  std::vector<Vec2> waypoints;
  std::string command;
  for (const auto& arg : arguments) {
    if (arg.substr(0, 2) == "--") {
      command = arg;
    } else {
      if (command == "--waypoints") {
        waypoints.push_back(to_vec2(arg));
      } else if (command == "--smoothing") {
        smoothing = arg;
      } else if (command == "--curvature") {
        maximum_curvature = std::stod(arg);
      } else if (command == "--radius") {
        radius = std::stod(arg);
      } else if (command == "--num-samples") {
        num_samples = std::stoi(arg);
      }
    }
  }

  if (num_samples <= 0) {
    std::cerr << "Need to specify --num-samples n (> 0)";
    std::cerr << "Got " << num_samples << std::endl;
    return 1;
  }

  if (smoothing != "fermat" && smoothing != "circle" && smoothing != "line") {
    std::cerr << "Need to specify --smoothing [fermat|circle|line]\n";
    std::cerr << "Got " << smoothing << std::endl;
    return 1;
  }

  if (waypoints.size() == 0) {
    std::cerr << "No waypoints specified, use --waypoints 0,0 1,1 2.3,4 ..."
              << std::endl;
    return 1;
  }

  std::shared_ptr<Path> path;
  if (smoothing == "fermat") {
    path = Path::fermat_smoothing(waypoints, maximum_curvature);
  } else if (smoothing == "circle") {
    path = Path::circular_smoothing(waypoints, radius);
  } else if (smoothing == "line") {
    path = Path::straight_line_path(waypoints);
  }
  // const auto path = std::make_shared<PathSpiral>(Vec2(0, 0), 0, 30, -2, 0);

  // const auto path = Path::straight_line_path(waypoints);
  const auto points = path->sample(num_samples);
  for (const auto& p : points) {
    std::cout << p.X() << "," << p.Y() << "\n";
  }

  return 0;
}
