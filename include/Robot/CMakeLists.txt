cmake_minimum_required(VERSION 3.8)
project(Robot LANGUAGES CXX)

set(CMAKE_DEBUG_POSTFIX "d")




if(WITH_BULLET)
        file(GLOB SOURCE_FILES "ServoAxis.cpp" "ServoAxis.h")
        file(GLOB BULLET_SRC_FILES "bullet_Indy7.cpp" "bullet_Indy7.h")
        add_library(${PROJECT_NAME} SHARED STATIC ${SOURCE_FILES} ${BULLET_SRC_FILES})
elseif(WITH_RAISIM)
        file(GLOB SOURCE_FILES "ServoAxis.cpp" "ServoAxis.h")
        add_library(${PROJECT_NAME} SHARED STATIC ${SOURCE_FILES})
else()
        file(GLOB SOURCE_FILES "ServoAxis.cpp" "ServoAxis.h")
        add_library(${PROJECT_NAME} SHARED STATIC ${SOURCE_FILES})
endif()



add_library(sub::Robot ALIAS ${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(${PROJECT_NAME}
        INTERFACE ${PROJECT_SOURCE_DIR})

# Eigen3
find_package(Eigen3 3.0 REQUIRED NO_MODULE)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# jsoncpp
find_package(jsoncpp REQUIRED)
target_link_libraries(${PROJECT_NAME} jsoncpp_lib)
list(APPEND INCLUDE_DIRS_FOR_DYNAMIC_VIEW ${jSONCPP_INCLUDE_DIRS})

find_package(Poco REQUIRED COMPONENTS Foundation Util Zip Net)
if (Poco_FOUND)
    MESSAGE(STATUS "Poco_INCLUDE_DIRS = ${Poco_INCLUDE_DIRS}")
    MESSAGE(STATUS "Poco_LIBRARIES = ${Poco_LIBRARIES}")
    target_include_directories(${PROJECT_NAME} PUBLIC ${Poco_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${Poco_LIBRARIES})
endif ()
if(WITH_BULLET)
    add_compile_definitions(__BULLET__)
    find_package(Bullet REQUIRED)
    if (BULLET_FOUND)
        MESSAGE(STATUS "BULLET_INCLUDE_DIRS = ${BULLET_INCLUDE_DIRS}")
        MESSAGE(STATUS "BULLET_LIBRARIES = ${BULLET_LIBRARIES}")
        target_include_directories(${PROJECT_NAME} PUBLIC ${BULLET_INCLUDE_DIRS})
        target_link_libraries(${PROJECT_NAME} ${BULLET_LIBRARIES})
    endif()
endif()

install(TARGETS ${PROJECT_MANE}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)