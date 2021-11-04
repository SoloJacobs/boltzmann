#include <gtest/gtest.h>

#include "../src/matrix/matrix.hpp"

// Demonstrate some basic assertions.
TEST(MatrixBasicTest, ConstructorTest) {
  auto matrix = Sol::Matrix<int>(10, 10);
  // Expect equality.
  EXPECT_EQ(matrix(0, 0), 0);
}
