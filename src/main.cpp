#include "matrix/matrix.hpp"
#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
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
  enum Direction { NORTH, SOUTH, EAST, WEST };

  struct Position_T {
    Position_T() = default;
    Position_T(int x, int y) : x(x), y(y) {}
    int x;
    int y;
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

    int mass() const { return north + south + east + west; }
  };

  ParticleGraph(Sol::Matrix<Element_T> initial_matrix, int height, int width)
      : velocities(initial_matrix), height(height), width(width){};

  void Update() {
    collisionOperator();
    streamingOperator();
  }

  Sol::Matrix<int> GetMass() {
    Sol::Matrix<int> result(height, width);
    std::transform(velocities.begin(), velocities.end(), result.begin(),
                   [](const Element_T &elem) { return elem.mass(); });
    return result;
  }

private:
  std::array<Position_T, 4> Neighbours(Position_T vertex) {
    std::array<Position_T, 4> result;
    result[Direction::NORTH] =
        Position_T(vertex.x, (vertex.y + height - 1) % height);
    result[Direction::SOUTH] =
        Position_T(vertex.x, (vertex.y + height + 1) % height);
    result[Direction::EAST] =
        Position_T((vertex.x + width + 1) % width, vertex.y);
    result[Direction::WEST] =
        Position_T((vertex.x + width - 1) % width, vertex.y);
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

  void streamingOperator() {
    Sol::Matrix<Element_T> copied_velocities(velocities);
    std::for_each(velocities.begin(), velocities.end(),
                  [&copied_velocities, this](Element_T &elem) {
                    auto neighbours = Neighbours(elem.pos);
                    elem.north =
                        copied_velocities(neighbours[Direction::SOUTH].x,
                                          neighbours[Direction::SOUTH].y)
                            .north;
                    elem.south =
                        copied_velocities(neighbours[Direction::NORTH].x,
                                          neighbours[Direction::NORTH].y)
                            .south;
                    elem.east = copied_velocities(neighbours[Direction::WEST].x,
                                                  neighbours[Direction::WEST].y)
                                    .east;
                    elem.west = copied_velocities(neighbours[Direction::EAST].x,
                                                  neighbours[Direction::EAST].y)
                                    .west;
                  });
  }

  Sol::Matrix<Element_T> velocities;
  int height;
  int width;
};

using uchar = unsigned char;
cv::Mat to_render(Sol::Matrix<int> mass) {
  cv::Mat im(cv::Size(mass.num_cols(), mass.num_rows()), CV_8UC1);
  auto to_format = [](size_t i) {
    return static_cast<uchar>(i * std::numeric_limits<uchar>::max() / 4);
  };
  std::transform(mass.begin(), mass.end(), im.begin<uchar>(), to_format);
  return im;
}

constexpr int width = 99;
constexpr int height = 99;

bool random_bool() {
  auto x = std::rand();
  return x < RAND_MAX / 8;
}

int main() {
  using namespace std::chrono_literals;

  Sol::Matrix<ParticleGraph::Element_T> init(width, height);
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      init(x, y) = ParticleGraph::Element_T(ParticleGraph::Position_T(x, y),
                                            random_bool(), random_bool(),
                                            random_bool(), random_bool());
    }
  }

  init(width / 2, height / 2).north = true;
  init(width / 2, height / 2).south = true;
  init(width / 2, height / 2).east = true;
  init(width / 2, height / 2).west = true;
  ParticleGraph PG(init, width, height);

  cv::VideoWriter output;
  auto inImg = to_render(PG.GetMass());
  output.open("build/live1.mp4", cv::VideoWriter::fourcc('a', 'v', 'c', '1'),
              15.0, inImg.size(), false);

  for (size_t i = 0; i < 15 * 60; ++i) {
    auto vis = PG.GetMass();
    output.write(to_render(PG.GetMass()));
    PG.Update();
  }
  return 0;
}
