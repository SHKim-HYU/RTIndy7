cmake_minimum_required(VERSION 3.0)
FIND_PACKAGE(jsoncpp REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
include_directories(${JSONCPP_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src")

PROJECT(pybullet_cpp_indy7)

cmake_minimum_required(VERSION 3.12)

SUBDIRS(bullet3)
file(GLOB_RECURSE PYBULLET_INDY7 src/Indy7.cpp src/Indy7.h )
file(GLOB_RECURSE MR ../include/MR/modern_robotics.h ../include/MR/modern_robotics.cpp)
file(GLOB_RECURSE MR_Indy7 ../include/MR_sim/MR_Indy7.h ../include/MR_sim/MR_Indy7.cpp)

set(EXECUTABLES
    JointHinfControl
)


foreach(EXECUTABLE ${EXECUTABLES})
    add_executable(${EXECUTABLE} src/${EXECUTABLES}/main.cpp ${MR_Indy7} ${MR} ${PYBULLET_INDY7})
    target_link_libraries(${EXECUTABLE} PUBLIC pybullet_cpp)
    target_link_libraries(${EXECUTABLE} PUBLIC jsoncpp_lib)
endforeach(EXECUTABLE)







