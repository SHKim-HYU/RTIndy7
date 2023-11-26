cmake_minimum_required(VERSION 3.8)
project(LR LANGUAGES CXX)

set(CMAKE_DEBUG_POSTFIX "d")
set(BUILD_SHARED_LIBS FALSE)

file(GLOB SOURCE_FILES "*.cpp" "*.c" "*.h")

add_library(${PROJECT_NAME}  STATIC
            ${SOURCE_FILES}
            )

add_library(sub::LR ALIAS ${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
        PUBLIC ${PROJECT_SOURCE_DIR})

find_package(Eigen3 3.0 REQUIRED NO_MODULE)
find_package(jsoncpp REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen jsoncpp)
