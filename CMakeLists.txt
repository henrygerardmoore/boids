cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
project(game LANGUAGES CXX C)

add_definitions(-g)

enable_testing()

find_package(spdlog REQUIRED)
find_package(SFML REQUIRED)
find_package(nlohmann_json REQUIRED)

add_executable(boids boids.cpp)

target_link_libraries(boids PRIVATE nlohmann_json::nlohmann_json sfml-graphics
                                    spdlog::spdlog)
