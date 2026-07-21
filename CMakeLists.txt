cmake_minimum_required(VERSION 3.16)
project(rompecabezas_grafica LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(generator src/generator.cpp)
target_link_libraries(generator PRIVATE ${OpenCV_LIBS})
target_include_directories(generator PRIVATE ${OpenCV_INCLUDE_DIRS})

add_executable(solver src/solver.cpp)
target_link_libraries(solver PRIVATE ${OpenCV_LIBS})
target_include_directories(solver PRIVATE ${OpenCV_INCLUDE_DIRS})
