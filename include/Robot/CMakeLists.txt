cmake_minimum_required(VERSION 3.8)
project(Robot LANGUAGES CXX)

set(CMAKE_DEBUG_POSTFIX "d")


file(GLOB SOURCE_FILES "ServoAxis.cpp" "ServoAxis.h")

if(WITH_BULLET)
        file(GLOB BULLET_SRC_FILES "bullet_Indy7.cpp" "bullet_Indy7.h")
        list(APPEND SOURCE_FILES ${BULLET_SRC_FILES})
endif()

if(WITH_CASADI)
        file(GLOB CSINDY_SRC_FILES "CS_Indy7.cpp" "CS_Indy7.h")
        list(APPEND SOURCE_FILES ${CSINDY_SRC_FILES})
endif()


add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
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

if(WITH_CASADI)
    add_compile_definitions(__CASADI__)
    find_package(CASADI REQUIRED)
    if (CASADI_FOUND)
        MESSAGE(STATUS "CASADI_INCLUDE_DIRS = ${CASADI_INCLUDE_DIR}")
        MESSAGE(STATUS "CASADI_LIBRARIES = ${CASADI_LIBRARIES}")
        target_include_directories(${PROJECT_NAME} PUBLIC ${CASADI_INCLUDE_DIR})
        target_link_libraries(${PROJECT_NAME} ${CASADI_LIBRARIES})
    endif ()
endif ()

install(TARGETS ${PROJECT_MANE}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)