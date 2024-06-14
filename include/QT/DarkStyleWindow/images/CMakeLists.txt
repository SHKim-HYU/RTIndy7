cmake_minimum_required(VERSION 3.8)
project(images LANGUAGES CXX)

file(GLOB IMAGE_FILES "*.png")

foreach(IMAGE_FILES)
        file(COPY ${IMAGE_FILES} DESTINATION ${CMAKE_BINARY_DIR}/images)
endforeach()
