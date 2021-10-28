#include "matrix/matrix.hpp"
#include <algorithm>
#include <array>
#include <iostream>
#include <limits>
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
  enum Direction { NORTH = 0, SOUTH = 1, WEST = 2, EAST = 3 };

  ParticleGraph(Sol::Matrix<bool> initial_matrix, size_t height, size_t width)
      : velocities(initial_matrix), height(height), width(width){};

  void Update() {
    collisionOperator();
    streamingOperator();
  }

  Sol::Matrix<size_t> GetMass() {
    Sol::Matrix<size_t> result(height, width);
    auto it = result.begin();
    for (size_t index = 0; index < velocities.num_cols(); ++index, ++it) {
      *it = velocities(NORTH, index) + velocities(SOUTH, index) +
            velocities(WEST, index) + velocities(EAST, index);
    }
    return result;
  }

private:
  std::array<size_t, 4> Neighbours(size_t vertex) {
    // row major
    auto y = vertex / width;
    auto x = vertex % width;

    std::array<std::pair<int, int>, 4> intermediate_result{};
    intermediate_result[Direction::EAST] = std::make_pair(x + 1, y);
    intermediate_result[Direction::WEST] = std::make_pair(x - 1, y);
    intermediate_result[Direction::NORTH] = std::make_pair(x, y + 1);
    intermediate_result[Direction::SOUTH] = std::make_pair(x, y - 1);

    auto apply_boundary_condition = [this](const std::pair<int, int> &coords) {
      std::pair<int, int> result;
      result.first =
          (coords.first + static_cast<int>(width)) % static_cast<int>(width);
      result.second =
          (coords.second + static_cast<int>(width)) % static_cast<int>(width);
      return result;
    };

    std::array<std::pair<int, int>, 4> intermediate_result2{};
    std::transform(intermediate_result.cbegin(), intermediate_result.cend(),
                   intermediate_result2.begin(), apply_boundary_condition);

    auto coord2idx = [this](std::pair<int, int> coords) {
      return coords.second * width + coords.first;
    };

    std::array<size_t, 4> result;
    std::transform(intermediate_result2.cbegin(), intermediate_result2.cend(),
                   result.begin(), coord2idx);

    return result;
  }

  void collisionOperator() {
    for (size_t index = 0; index <= velocities.num_cols(); ++index) {
      std::swap(velocities(Direction::NORTH, index),
                velocities(Direction::SOUTH, index));
      std::swap(velocities(Direction::WEST, index),
                velocities(Direction::EAST, index));
    }
  }

  void streamingOperator() {
    Sol::Matrix<bool> copied_velocities(velocities);
    for (size_t index = 0; index <= velocities.num_cols(); ++index) {
      auto neighbours = Neighbours(index);
      velocities(Direction::SOUTH, index) =
          copied_velocities(Direction::NORTH, neighbours[Direction::SOUTH]);
      velocities(Direction::NORTH, index) =
          copied_velocities(Direction::SOUTH, neighbours[Direction::NORTH]);
      velocities(Direction::WEST, index) =
          copied_velocities(Direction::EAST, neighbours[Direction::WEST]);
      velocities(Direction::EAST, index) =
          copied_velocities(Direction::WEST, neighbours[Direction::EAST]);
    }
  }

  Sol::Matrix<bool> velocities;
  size_t height;
  size_t width;
};

using uchar = unsigned char;
cv::Mat to_render(Sol::Matrix<size_t> mass) {
  cv::Mat im(cv::Size(mass.num_cols(), mass.num_rows()), CV_8UC1);
  auto to_format = [](size_t i) {
    return static_cast<uchar>(i * std::numeric_limits<uchar>::max() / 4);
  };
  std::transform(mass.begin(), mass.end(), im.begin<uchar>(), to_format);
  return im;
}

constexpr size_t width = 99;
constexpr size_t height = 99;

int main() {
  using namespace std::chrono_literals;

  Sol::Matrix<bool> init(4, width * height);
  init(ParticleGraph::NORTH, 55) = true;
  init(ParticleGraph::SOUTH, 55) = true;
  init(ParticleGraph::EAST, 33) = true;
  init(ParticleGraph::WEST, 33) = true;
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
