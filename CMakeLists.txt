cmake_minimum_required(VERSION 3.16)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -pedantic -Werror -O3 -std=c++17)

project(Name LANGUAGES CXX)
add_subdirectory(test)
add_subdirectory(src)
add_subdirectory(benchmark)
