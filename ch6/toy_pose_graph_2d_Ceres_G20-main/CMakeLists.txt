cmake_minimum_required(VERSION 3.10)
project(PoseGraphDemo)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)

set(CMAKE_CXX_STANDARD 14)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(${EIGEN3_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS} ${G2O_INCLUDE_DIRS})

set(G2O_LIBS
	/usr/local/lib/libg2o_core.so 
	/usr/local/lib/libg2o_types_sba.so
	/usr/local/lib/libg2o_types_slam3d.so
	/usr/local/lib/libg2o_types_slam2d.so
	/usr/local/lib/libg2o_solver_csparse.so
	/usr/local/lib/libg2o_stuff.so
	/usr/local/lib/libg2o_csparse_extension.so
	)


add_executable(PoseGraphDemoCeres ceresBackend.cpp g2oDataManager.cpp ceres_solve/problemPoseGraph.cpp)
target_link_libraries(PoseGraphDemoCeres ${CERES_LIBRARIES})

add_executable(PoseGraphDemoG2O g2oBackend.cpp g2oDataManager.cpp)
target_link_libraries(PoseGraphDemoG2O ${G2O_LIBS})

