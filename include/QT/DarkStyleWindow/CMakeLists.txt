cmake_minimum_required(VERSION 3.8)
project(DarkStyleWindow LANGUAGES CXX)

set(CMAKE_DEBUG_POSTFIX "d")
file(GLOB SOURCE_FILES "*.cpp" "*.c" "*.h")

# Find the QtWidgets library
find_package(Qt5 COMPONENTS Widgets Core REQUIRED)

add_subdirectory(darkstyle)
add_subdirectory(framelesswindow)
add_subdirectory(images)

add_library(${PROJECT_NAME} SHARED STATIC
            ${SOURCE_FILES} 
            )

add_library(sub::DarkStyleWindow ALIAS ${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(${PROJECT_NAME}
        INTERFACE ${PROJECT_SOURCE_DIR} 
        ${PCAN_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}
    sub::framelesswindow
)

# Use the Widgets module from Qt 5
target_link_libraries(${PROJECT_NAME} 
    Qt5::Widgets 
    Qt5::Core
    Qt5::Gui
    Qt5::Network
    Qt5::Quick
)

install(TARGETS ${PROJECT_MANE}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)