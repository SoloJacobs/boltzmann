#include "matrix/matrix.hpp"
#include <algorithm>
#include <array>
#include <iostream>
#include <thread>
#include <vector>

struct Point_2D {
  double x;
  double y;
};
enum class Direction { NORTH = 0, SOUTH = 1, WEST = 2, EAST = 3 };
constexpr size_t Num_velocities = 4;

struct Element {
  std::array<bool, Num_velocities> hasVelocity;
  Point_2D coordinates;
  bool &North() { return hasVelocity[0]; }
  bool &South() { return hasVelocity[1]; }
  bool &West() { return hasVelocity[2]; }
  bool &East() { return hasVelocity[3]; }
};

struct Grid {

  Grid(size_t column_length, size_t row_width) : row_width(row_width) {
    elements = std::vector<Element>((row_width + 2) * (column_length + 2));
    Element boundary{std::array<bool, 4>{0, 0, 0, 0}, Point_2D{0., 0.}};
    std::fill(elements.begin(), elements.begin() + row_width + 2, boundary);
    std::fill(elements.rbegin(), elements.rbegin() + row_width + 2, boundary);
    for (auto i = row_width + 2; i < elements.size(); i += row_width + 2) {
      elements[i] = boundary;
      elements[i - 1] = boundary;
    }
  }

  size_t to_underlying(Direction d) { return static_cast<size_t>(d); }

  std::vector<Element> elements;
  size_t row_width;

  std::array<size_t, Num_velocities> streamingOperatorMask(size_t index) {
    return {index - (row_width + 2), index + (row_width + 2), index - 1,
            index + 1};
  }

  std::array<Element, Num_velocities>
  getMaskedElements(std::array<size_t, Num_velocities> mask) {
    std::array<Element, Num_velocities> result;
    std::transform(mask.begin(), mask.end(), result.begin(),
                   [this](const auto index) { return elements[index]; });
    return result;
  }

  Element streamingOperator(size_t index,
                            const std::vector<Element> &elements) {
    Element result = elements[index];
    auto relevant_indices = streamingOperatorMask(index);
    auto relevant_elements = getMaskedElements(relevant_indices);
    result.North() = relevant_elements[to_underlying(Direction::SOUTH)].North();
    result.South() = relevant_elements[to_underlying(Direction::NORTH)].South();
    result.East() = relevant_elements[to_underlying(Direction::WEST)].East();
    result.West() = relevant_elements[to_underlying(Direction::EAST)].West();
    return result;
  }

  std::vector<double> getGrid() const {
    std::vector<double> result{};
    for (const auto &elem : elements) {
      double newVal = 0;
      for (const auto &velo : elem.hasVelocity) {
        newVal += velo ? 0.25 : 0.0;
      }
      result.emplace_back(newVal);
    }
    return result;
  }
};

void render(size_t width, size_t height, std::vector<double> values) {
  for (auto h = 0; h < height; ++h) {
    for (auto w = 0; w < width; ++w) {
      // val = values(h, w)
      auto val = values[h * width + w];
      if (val > 0.5) {
        std::cout << 'o';
      } else {
        std::cout << ' ';
      }
    }
    std::cout << '\n';
  }
}

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
  ParticleGraph(Matrix<bool> initial_matrix) : velocities(initial_matrix) {
    height = velocities.num_rows();
    width = velocities.num_cols();
  };

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

    std::array<std::pair<size_t, size_t>, 4> intermediate_result{};
    intermediate_result[Direction::EAST] = std::make_pair(x + 1, y);
    intermediate_result[Direction::WEST] = std::make_pair(x - 1, y);
    intermediate_result[Direction::NORTH] = std::make_pair(x, y + 1);
    intermediate_result[Direction::SOUTH] = std::make_pair(x, y - 1);

    auto apply_boundary_condition = [this](std::pair<size_t, size_t> coords) {
      coords.first = coords.first % width;
      coords.second = coords.second % width;
      return coords;
    };

    std::transform(intermediate_result.begin(), intermediate_result.end(),
                   intermediate_result.begin(), apply_boundary_condition);

    auto coord2idx = [this](std::pair<size_t, size_t> coords) {
      return coords.second * width + coords.first;
    };

    std::array<size_t, 4> result;
    std::transform(intermediate_result.cbegin(), intermediate_result.cend(),
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

  enum Direction { NORTH = 0, SOUTH = 1, WEST = 2, EAST = 3 };

  Matrix<bool> velocities; //(4, width * height)
  size_t height;
  size_t width;
};

int main() {
  using namespace std::chrono_literals;

  // Grid test_grid(10, 10);
  // Matrix<int> a(10, 10);
  // Matrix<int> b(10, 10);
  // b(0, 0) = 1;
  // Matrix<int> c(10, 10);

  Matrix<bool> init(4, 10 * 10);
  init(0, 0) = true;
  ParticleGraph PG(init);
  auto vis = PG.GetMass();

  std::cout << vis;
  // for (size_t i = 0; i < 10; ++i) {
  //   clear();
  //   std::this_thread::sleep_for(0.5s);
  // }
  // Matrix<bool> d(10, 10);
  // std::cout << d;
  return 0;
}
