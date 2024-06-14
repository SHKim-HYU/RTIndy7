cmake_minimum_required(VERSION 3.8)
project(darkstyle LANGUAGES CXX)

file(GLOB STYLE_FILES "*.qss" "*.png")

foreach(STYLE_FILES)
        file(COPY ${STYLE_FILE} DESTINATION ${CMAKE_BINARY_DIR}/darkstyle)
endforeach()
