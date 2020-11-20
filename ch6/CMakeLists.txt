cmake_minimum_required(VERSION 3.10)
project(Fittings)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(G2O REQUIRED)

set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${G2O_INCLUDE_DIRS})

add_executable(LinearFitting linear_fitting.cpp)

add_executable(curveFitting  curve_fitting.cpp)

add_executable(curveFittingCeres curve_fitting_ceres.cpp)
target_link_libraries(curveFittingCeres ${CERES_LIBRARIES})


add_executable(curveFittingG2O  curve_fitting_g2o.cpp)
target_link_libraries(curveFittingG2O  g2o_core g2o_stuff)
