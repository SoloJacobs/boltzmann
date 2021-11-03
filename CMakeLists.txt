cmake_minimum_required(VERSION 3.14)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra -pedantic -Werror)

project(Name LANGUAGES CXX)
add_subdirectory(src)
add_subdirectory(test)
