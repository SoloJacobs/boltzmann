#include "matrix/matrix.hpp"
#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
#include <optional>
#include <random>
#include <thread>
#include <vector>

#include <opencv4/opencv2/core/mat.hpp>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/videoio.hpp>

// template <typename Result_container_t>
// void plot(size_t width, size_t height, Result_container_t cont, double dt) {
//   for (const auto &iteration : cont) {
//     render(width, height, iteration);
//     std::cout << "\n====\n";
//   }
// }

//       N W S O
// (0,0) 0 0 0 0
// (0,1) 0 1 1 0
// (1,0) 0 0 0 1
// (1,1) 0 0 0 0

// neighbours
// 1 2
//

// 0 2
// 1 0

// class ParticleGraph1 {
// 	struct Particle {
// 		Point_2D coord;
// 		std::vector<bool> velocities(4);
// 		std::vector<Particle*> neighbours;
// 	}
// 	std::vector<Particle> list;
// }

class ParticleGraph {

public:
  enum class Direction { N, NE, E, SE, S, SW, W, NW };
  enum class BoundaryType { PERIODIC, REFLECTING };

  struct Position_T {
    Position_T() = default;
    Position_T(size_t x, size_t y) : x(x), y(y) {}
    size_t x;
    size_t y;
  };

  struct Boundary_T {
    Boundary_T(Direction direction, BoundaryType type)
        : direction(direction), type(type) {}
    Direction direction;
    BoundaryType type;
  };

  struct Neighbour_T {
    Position_T north;
    Position_T south;
    Position_T east;
    Position_T west;
    std::optional<Boundary_T> boundary;
  };

  struct Element_T {

    Element_T() = default;
    Element_T(Position_T pos)
        : pos(pos), north(false), south(false), east(false), west(false) {}
    Element_T(Position_T pos, bool north, bool south, bool east, bool west)
        : pos(pos), north(north), south(south), east(east), west(west) {}

    Position_T pos;
    bool north;
    bool south;
    bool east;
    bool west;

    size_t mass() const { return north + south + east + west; }
  };

  ParticleGraph(Sol::Matrix<Element_T> initial_matrix, size_t height,
                size_t width, BoundaryType boundary_type)
      : velocities(initial_matrix), height(height), width(width),
        neighbours_cache(Sol::Matrix<Neighbour_T>(width, height)),
        boundary_type(boundary_type) {
    std::for_each(
        velocities.begin(), velocities.end(), [this](const Element_T &elem) {
          neighbours_cache(elem.pos.x, elem.pos.y) = Neighbours(elem.pos);
        });
  };

  void Update() {
    streamingOperator_reflect();
    collisionOperator();
  }

  Sol::Matrix<size_t> GetMass() {
    Sol::Matrix<size_t> result(height, width);
    std::transform(velocities.begin(), velocities.end(), result.begin(),
                   [](const Element_T &elem) { return elem.mass(); });
    return result;
  }

private:
  Neighbour_T Neighbours(const Position_T &vertex) {
    Neighbour_T result;

    const auto &x = vertex.x;
    const auto &y = vertex.y;

    result.north = Position_T(x, (y + height - 1) % height);
    result.south = Position_T(x, (y + height + 1) % height);
    result.east = Position_T((x + width + 1) % width, y);
    result.west = Position_T((x + width - 1) % width, y);

    if (x == 0 && y == 0) {
      result.boundary = Boundary_T(Direction::NW, boundary_type);
      return result;
    }
    if (x == 0 && y == height) {
      result.boundary = Boundary_T(Direction::SW, boundary_type);
      return result;
    }
    if (x == 0) {
      result.boundary = Boundary_T(Direction::W, boundary_type);
      return result;
    }

    if (x == width && y == 0) {
      result.boundary = Boundary_T(Direction::NE, boundary_type);
      return result;
    }
    if (x == width && y == height) {
      result.boundary = Boundary_T(Direction::SE, boundary_type);
      return result;
    }
    if (x == width) {
      result.boundary = Boundary_T(Direction::E, boundary_type);
      return result;
    }

    if (y == 0) {
      result.boundary = Boundary_T(Direction::N, boundary_type);
      return result;
    }
    if (y == height) {
      result.boundary = Boundary_T(Direction::S, boundary_type);
      return result;
    }

    return result;
  }

  void collisionOperator() {
    std::for_each(velocities.begin(), velocities.end(), [](Element_T &elem) {
      if ((elem.north && elem.south) || (elem.east && elem.west)) {
        std::swap(elem.north, elem.east);
        std::swap(elem.west, elem.south);
      }
    });
  }

  void streamingOperator_reflect() {
    const Sol::Matrix<Element_T> copied_velocities(velocities);
    std::for_each(
        velocities.begin(), velocities.end(),
        [&copied_velocities, this](Element_T &elem) {
          Neighbour_T &neighbours = neighbours_cache(elem.pos.x, elem.pos.y);
          switch (neighbours.boundary->direction) {
          case Direction::N:
            elem.south = elem.north;
            elem.north =
                copied_velocities(neighbours.south.x, neighbours.south.y).north;
            elem.east =
                copied_velocities(neighbours.west.x, neighbours.west.y).east;
            elem.west =
                copied_velocities(neighbours.east.x, neighbours.east.y).west;
            break;
          case Direction::NE:
            elem.south = elem.north;
            elem.north =
                copied_velocities(neighbours.south.x, neighbours.south.y).north;
            elem.east =
                copied_velocities(neighbours.west.x, neighbours.west.y).east;
            elem.west = elem.east;
            break;
          case Direction::E:
            elem.west = elem.east;
            elem.east =
                copied_velocities(neighbours.west.x, neighbours.west.y).east;
            elem.north =
                copied_velocities(neighbours.south.x, neighbours.south.y).north;
            elem.south =
                copied_velocities(neighbours.north.x, neighbours.north.y).south;
            break;
          case Direction::SE:
            elem.north = elem.south;
            elem.south =
                copied_velocities(neighbours.north.x, neighbours.north.y).south;
            elem.west = elem.east;
            elem.east =
                copied_velocities(neighbours.west.x, neighbours.west.y).east;
            break;
          case Direction::S:
            elem.north = elem.south;
            elem.south =
                copied_velocities(neighbours.north.x, neighbours.north.y).south;
            elem.east =
                copied_velocities(neighbours.west.x, neighbours.west.y).east;
            elem.west =
                copied_velocities(neighbours.east.x, neighbours.east.y).west;
            break;
          case Direction::SW:
            elem.north = elem.south;
            elem.south =
                copied_velocities(neighbours.north.x, neighbours.north.y).south;
            elem.east = elem.west;
            elem.west =
                copied_velocities(neighbours.east.x, neighbours.east.y).west;
            break;
          case Direction::W:
            elem.east = elem.west;
            elem.west =
                copied_velocities(neighbours.east.x, neighbours.east.y).west;
            elem.north =
                copied_velocities(neighbours.south.x, neighbours.south.y).north;
            elem.south =
                copied_velocities(neighbours.north.x, neighbours.north.y).south;
            break;
          case Direction::NW:
            elem.east = elem.west;
            elem.west =
                copied_velocities(neighbours.east.x, neighbours.east.y).west;
            elem.south = elem.north;
            elem.north =
                copied_velocities(neighbours.south.x, neighbours.south.y).north;
            break;
          default:
            elem.north =
                copied_velocities(neighbours.south.x, neighbours.south.y).north;
            elem.south =
                copied_velocities(neighbours.north.x, neighbours.north.y).south;
            elem.east =
                copied_velocities(neighbours.west.x, neighbours.west.y).east;
            elem.west =
                copied_velocities(neighbours.east.x, neighbours.east.y).west;
            break;
          }
        });
  }

  void streamingOperator() {
    const Sol::Matrix<Element_T> copied_velocities(velocities);
    std::for_each(
        velocities.begin(), velocities.end(),
        [&copied_velocities, this](Element_T &elem) {
          Neighbour_T &neighbours = neighbours_cache(elem.pos.x, elem.pos.y);
          elem.north =
              copied_velocities(neighbours.south.x, neighbours.south.y).north;
          elem.south =
              copied_velocities(neighbours.north.x, neighbours.north.y).south;
          elem.east =
              copied_velocities(neighbours.west.x, neighbours.west.y).east;
          elem.west =
              copied_velocities(neighbours.east.x, neighbours.east.y).west;
        });
  }

  Sol::Matrix<Element_T> velocities;
  size_t height;
  size_t width;
  Sol::Matrix<Neighbour_T> neighbours_cache;
  BoundaryType boundary_type;
};

using uchar = unsigned char;
cv::Mat to_render(const Sol::Matrix<size_t> &mass) {
  cv::Mat im(cv::Size(mass.num_cols(), mass.num_rows()), CV_8UC1);
  auto to_format = [](size_t i) {
    return static_cast<uchar>(i * std::numeric_limits<uchar>::max() / 4);
  };
  std::transform(mass.begin(), mass.end(), im.begin<uchar>(), to_format);
  return im;
}

constexpr size_t width = 400;
constexpr size_t height = 400;
constexpr double fps = 30;
constexpr size_t duration_s = 20;

bool random_bool() {
  auto x = std::rand();
  return x < RAND_MAX / 3;
}

int main() {
  using namespace std::chrono_literals;

  Sol::Matrix<ParticleGraph::Element_T> init(height, width);
  for (size_t x = 0; x < width; ++x) {
    for (size_t y = 0; y < height; ++y) {
      if (!(x > width / 16 && x < width / 2 && y > height / 16 &&
            y < height / 2))
        init(x, y) = ParticleGraph::Element_T(ParticleGraph::Position_T(x, y),
                                              random_bool(), random_bool(),
                                              random_bool(), random_bool());
      else
        init(x, y) = ParticleGraph::Element_T(ParticleGraph::Position_T(x, y));
    }
  }
  ParticleGraph PG(init, width, height, ParticleGraph::BoundaryType::PERIODIC);

  cv::VideoWriter output;
  auto inImg = to_render(PG.GetMass());
  output.open("build/live1.mp4", cv::VideoWriter::fourcc('a', 'v', 'c', '1'),
              fps, inImg.size(), false);

  for (size_t i = 0; i < static_cast<size_t>(fps * duration_s); ++i) {
    std::cout << i << std::endl;
    auto vis = PG.GetMass();
    output.write(to_render(PG.GetMass()));
    PG.Update();
  }
  return 0;
}
