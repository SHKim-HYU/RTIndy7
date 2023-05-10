cmake_minimum_required(VERSION 3.0)
FIND_PACKAGE(jsoncpp REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
include_directories(${JSONCPP_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")

PROJECT(pybullet_cpp_indy7)

cmake_minimum_required(VERSION 3.12)

SUBDIRS(bullet3)
file(GLOB_RECURSE PYBULLET_INDY7 src/Indy7.cpp src/Indy7.h )
file(GLOB_RECURSE MR ../include/MR/modern_robotics.h ../include/MR/modern_robotics.cpp)
file(GLOB_RECURSE MR_Indy7 ../include/MR/MR_Indy7.h ../include/MR/MR_Indy7.cpp)

add_executable(pybullet_cpp_indy7 src/main.cpp ${MR_Indy7} ${MR} ${PYBULLET_INDY7} )

target_link_libraries(pybullet_cpp_indy7 PUBLIC pybullet_cpp)
target_link_libraries(pybullet_cpp_indy7 PUBLIC jsoncpp_lib)



