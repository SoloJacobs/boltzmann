find_package(benchmark REQUIRED)

add_executable(bm bench.cpp)
target_link_libraries(bm PRIVATE benchmark)
