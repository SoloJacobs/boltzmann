enable_testing()

find_package(GTest REQUIRED)
message(STATUS "FOUND GTEST: ${GTEST_FOUND}")

add_executable(
  all_tests
  matrix_test.cpp
)
target_link_libraries(
  all_tests
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(all_tests)

