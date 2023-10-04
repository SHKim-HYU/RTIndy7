cmake_minimum_required(VERSION 3.8)
project(CAN LANGUAGES CXX)

set(CMAKE_DEBUG_POSTFIX "d")
file(GLOB SOURCE_FILES "*.cpp" "*.c" "*.h")

find_path(PCAN_INCLUDE_DIR NAMES libpcanfd.h)
find_library(PCAN_LIBRARY NAMES pcanfd)

add_library(${PROJECT_NAME} SHARED STATIC
            ${SOURCE_FILES} 
            )

add_library(sub::CAN ALIAS ${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(${PROJECT_NAME}
        INTERFACE ${PROJECT_SOURCE_DIR} ${PCAN_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PCAN_LIBRARY})

install(TARGETS ${PROJECT_MANE}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)