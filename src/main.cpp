#include "matrix/matrix.hpp"
#include <algorithm>
#include <array>
#include <iostream>
#include <thread>
#include <vector>

template <typename Result_container_t>
void plot(size_t width, size_t height, Result_container_t cont, double dt) {
  for (const auto &iteration : cont) {
    render(width, height, iteration);
    std::cout << "\n====\n";
  }
}

void clear() { std::cout << "\x1B[2J\x1B[H"; }

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

  ParticleGraph(Matrix<bool> initial_matrix, size_t height, size_t width)
      : velocities(initial_matrix), height(height), width(width){};

  void Update() {
    collisionOperator();
    streamingOperator();
  }

  Matrix<size_t> GetMass() {
    Matrix<size_t> result(height, width);
    auto it = result.begin();
    for (auto index = 0; index < velocities.num_cols(); ++index, ++it) {
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
    for (auto index = 0; index <= velocities.num_cols(); ++index) {
      std::swap(velocities(Direction::NORTH, index),
                velocities(Direction::SOUTH, index));
      std::swap(velocities(Direction::WEST, index),
                velocities(Direction::EAST, index));
    }
  }

  void streamingOperator() {
    Matrix<bool> copied_velocities(velocities);
    for (auto index = 0; index <= velocities.num_cols(); ++index) {
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

  Matrix<bool> velocities;
  size_t height;
  size_t width;
};

int main() {
  using namespace std::chrono_literals;

  Matrix<bool> init(4, 10 * 10);
  init(ParticleGraph::NORTH, 55) = true;
  init(ParticleGraph::SOUTH, 55) = true;
  ParticleGraph PG(init, 10, 10);

  for (size_t i = 0; i < 25; ++i) {
    clear();
    auto vis = PG.GetMass();
    std::cout << vis;
    std::this_thread::sleep_for(0.3s);
    PG.Update();
  }
  return 0;
}
