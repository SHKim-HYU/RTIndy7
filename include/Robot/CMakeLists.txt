cmake_minimum_required(VERSION 3.8)
project(Robot LANGUAGES CXX)

set(CMAKE_DEBUG_POSTFIX "d")


file(GLOB SOURCE_FILES "*.cpp" "*.h" "*.c")

#bullet3
set(BULLET_PHYSICS_SOURCE_DIR  "/opt/bullet3")
include_directories(
    ${BULLET_PHYSICS_SOURCE_DIR}/src
    ${BULLET_PHYSICS_SOURCE_DIR}/examples
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator		
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/enet/include
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/ThirdPartyLibs/clsocket/src
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/CommonInterfaces
)
SET(RobotSimulator_SRCS_GUI
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/b3RobotSimulatorClientAPI.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/b3RobotSimulatorClientAPI.h	
    ${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/MinitaurSetup.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/RobotSimulator/MinitaurSetup.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/ExampleBrowser/InProcessExampleBrowser.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/GraphicsServerExample.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/GraphicsClientExample.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/RemoteGUIHelper.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/GraphicsServerExample.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/GraphicsClientExample.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/RemoteGUIHelper.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/GraphicsSharedMemoryCommands.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/GraphicsSharedMemoryPublic.h
	${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/PhysicsServerExample.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/PhysicsServerExampleBullet2.cpp
	${BULLET_PHYSICS_SOURCE_DIR}/examples/SharedMemory/SharedMemoryInProcessPhysicsC_API.cpp	
)

add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES} ${RobotSimulator_SRCS_GUI} )
add_library(sub::Robot ALIAS ${PROJECT_NAME})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(${PROJECT_NAME}
        INTERFACE ${PROJECT_SOURCE_DIR})

# Eigen3
find_package(Eigen3 3.0 REQUIRED NO_MODULE)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen ${LAPACK_LIBRARIES})

# jsoncpp
find_package(jsoncpp REQUIRED)
target_link_libraries(${PROJECT_NAME} jsoncpp_lib)
list(APPEND INCLUDE_DIRS_FOR_DYNAMIC_VIEW ${jSONCPP_INCLUDE_DIRS})

find_package(Poco REQUIRED COMPONENTS Foundation Util Zip Net)
MESSAGE(STATUS "Poco_INCLUDE_DIRS = ${Poco_INCLUDE_DIRS}")
MESSAGE(STATUS "Poco_LIBRARIES = ${Poco_LIBRARIES}")
target_include_directories(${PROJECT_NAME} PUBLIC ${Poco_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Poco_LIBRARIES})

add_compile_definitions(__CASADI__)
find_package(CASADI REQUIRED)
if (CASADI_FOUND)
    MESSAGE(STATUS "CASADI_INCLUDE_DIRS = ${CASADI_INCLUDE_DIR}")
    MESSAGE(STATUS "CASADI_LIBRARIES = ${CASADI_LIBRARIES}")
    target_include_directories(${PROJECT_NAME} PUBLIC ${CASADI_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} ${CASADI_LIBRARIES} dl)
endif ()


add_compile_definitions(__BULLET__)
find_package(Bullet REQUIRED)
if (BULLET_FOUND)
    MESSAGE(STATUS "BULLET_INCLUDE_DIRS = ${BULLET_INCLUDE_DIRS}")
    MESSAGE(STATUS "BULLET_LIBRARIES = ${BULLET_LIBRARIES}")
    target_include_directories(${PROJECT_NAME} PUBLIC ${BULLET_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${BULLET_LIBRARIES})
endif()
target_compile_definitions(${PROJECT_NAME} PUBLIC -DBT_USE_DOUBLE_PRECISION)	

install(TARGETS ${PROJECT_MANE}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)