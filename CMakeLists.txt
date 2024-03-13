###
# Set minimum version of CMake. Since command 'project' use
# VERSION sub-option we need at least 3.0.
# Note: If you use 2.6 or 2.4, God kills a kitten. Seriously.
cmake_minimum_required(VERSION 3.16)

# THIS HAS TO COME BEFORE THE PROJECT LINE
#set(CMAKE_C_COMPILER "gcc")
#set(CMAKE_CXX_COMPILER "gcc")

####
# Set variables:
#   * PROJECT_NAME
#   * PROJECT_VERSION
project(RT_Indy7 VERSION 2.0.0 LANGUAGES C CXX)

# Use MACOSX_RPATH by default on OS X. This was added in CMake 2.8.12 and
# became default in CMake 3.0. Explicitly setting this policy is necessary to
# suppress a warning in CMake 3.0 and above.
if(POLICY CMP0042)
    cmake_policy(SET CMP0042 NEW)
endif()

# Simplify variable reference and escape sequence evaluation. This was added in
# CMake 3.1. Explicitly setting this policy is necessary to suppress a warning
# in CMake 3.1 and above.
if(POLICY CMP0053)
    cmake_policy(SET CMP0053 NEW)
endif()

cmake_policy(SET CMP0022 NEW)
if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif()

if (POLICY CMP0025)
    cmake_policy(SET CMP0025 NEW)
endif()

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(FindPkgConfig)
include(modules/CMakeFunctions.cmake)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})
set(PROJ_INSTALL_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/bin/inc)
set(PROJ_INSTALL_BIN_DIR ${PROJECT_SOURCE_DIR}/bin)
set(PROJ_INSTALL_LIB_DIR ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_DEBUG_POSTFIX "d")
set(CMAKE_MODULE_PATH  ${CMAKE_CURRENT_SOURCE_DIR}/modules)
set(BUILD_SHARED_LIBS TRUE)

#===============================================================================
# Build options
#===============================================================================
#option(BUILD_DOC "Build Documentation" off)
option(RTIndy7_VERBOSE "Whether print detailed information in CMake process" ON)
option(RTIndy7_SHARED_LIBS "Build shared libraries" ON)
option(WITH_RP "with Indy RP option" ON)
option(WITH_CB "with Indy7 ControlBox" ON)
option(WITH_DUALARM "DUALARM" ON)
option(WITH_CASADI "import for CasADi function" OFF)
option(WITH_POCO "with POCO Comunication" OFF)
option(WITH_BULLET "with Bullet Simulation" ON)
option(WITH_RAISIM "with RAISIM Simulation" OFF)
option(WITH_QT "with QT app" ON)
option(WITH_LR "with LR Control" ON)
#===============================================================================
# Build type settings
#===============================================================================
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPERCASE)

set(BUILD_TYPE_DEBUG TRUE)
set(BUILD_TYPE_RELEASE TRUE)
set(BUILD_TYPE_RELWITHDEBINFO FALSE)
set(BUILD_TYPE_MINSIZEREL FALSE)

if("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "DEBUG")
    set(BUILD_TYPE_DEBUG TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELEASE")
    set(BUILD_TYPE_RELEASE TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELWITHDEBINFO")
    set(BUILD_TYPE_RELWITHDEBINFO TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "MINSIZEREL")
    set(BUILD_TYPE_MINSIZEREL TRUE)
else()
    message(WARNING "CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} unknown. Valid options are: Debug | Release | RelWithDebInfo | MinSizeRel")
endif()

#===============================================================================
# Compiler flags
#===============================================================================
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-std=c++17 CXX17_SUPPORT)
if(CXX17_SUPPORT)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
else()
    check_cxx_compiler_flag(-std=c++14 CXX14_SUPPORT)
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14" )
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC" )
execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -dumpfullversion -dumpversion OUTPUT_VARIABLE GCC_VERSION)
if(GCC_VERSION VERSION_LESS 5.3.1)
    message(FATAL_ERROR "The installed g++ version is ${GCC_VERSION}. ${PROJECT_NAME} requires g++ 5.3.1 or greater.")
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")

# set(KDL_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/KDL)
set(ECAT_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/EcatSystem)
# set(CAN_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/CAN)
set(INTP_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/Interpolator)
set(NET_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/Network)
set(ROBOT_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/Robot)


# add_subdirectory(include/KDL)
add_subdirectory(include/EcatSystem)
# add_subdirectory(include/CAN)
add_subdirectory(include/Interpolator)
add_subdirectory(include/Network)
add_subdirectory(include/Robot)



# Find package
find_package(Qt5 COMPONENTS Widgets Core Quick REQUIRED)
find_package(Eigen 3.0.0)

find_package(jsoncpp REQUIRED)
find_package(Xenomai 3.1.1 REQUIRED ALCHEMY)
find_package(cxxopts REQUIRED)





SET(CONTROL_SRCS src/control/control_run.cpp )
SET(PRINT_SRCS src/print/print_run.cpp )
SET(ECAT_SRCS  src/ecat/ecat_run.cpp )
SET(SAFETY_SRCS  src/safety/safety_run.cpp )
SET(TCP_SRCS  src/tcp/tcp_run.cpp )
SET(LR_CONTROL_SRCS  src/lr_control/lr_control_run.cpp )
SET(BULLET_SRCS src/bullet/bullet_run.cpp)
SET(FT_SRCS  src/ft/ft_run.cpp  src/ft/FTread.cpp )

SET(COMMON_SRCS ${PRINT_SRCS} ${ECAT_SRCS} ${SAFETY_SRCS} ${TCP_SRCS}  )
SET(COMMON_INCLUDE_DIRS ${ECAT_HEADERS_DIR} ${ROBOT_HEADERS_DIR} ${NET_HEADERS_DIR} ${INTP_HEADERS_DIR})
SET(COMMON_LIBRARIES   sub::ECAT sub::Interpolator sub::NETWORK sub::Robot dl)
if(WITH_QT)
    add_compile_definitions(__QT__)
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTOUIC ON)
    set(CMAKE_AUTORCC ON)

    set(QTWIDGET_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/QT/QtWidgets)
    set(DARKSTYLE_HEADERS_DIR ${PROJECT_SOURCE_DIR}/include/QT/DarkStyleWindow)
    include_directories(
        ${QTWIDGET_HEADERS_DIR}
        ${DARKSTYLE_HEADERS_DIR}    
    )
    add_subdirectory(include/QT/DarkStyleWindow)
    add_subdirectory(include/QT/QtWidgets)
    add_subdirectory(include/QT/QCustomPlot_library)
    set(QT_LIBRARIES 	sub::QtWidgets	sub::DarkStyleWindow	Qt5::Widgets 	Qt5::Core	Qt5::Gui	Qt5::Network	Qt5::Quick	qcustomplot  )
    SET(QT_SRCS src/qt/mainwindow.cpp src/qt/mainwindow.h src/qt/mainwindow.ui src/qt/qt_run.cpp src/qt/qt_run.h)
    SET(COMMON_SRCS ${COMMON_SRCS}  ${QT_SRCS})
    SET(COMMON_INCLUDE_DIRS ${COMMON_INCLUDE_DIRS} )
    SET(COMMON_LIBRARIES  ${COMMON_LIBRARIES} ${QT_LIBRARIES})
endif()
if(WITH_BULLET)
    find_package(Bullet REQUIRED)
    find_package(SimRobot REQUIRED)
    add_compile_definitions(__BULLET__)
    SET(COMMON_SRCS ${COMMON_SRCS} ${BULLET_SRCS} ${RobotSimulator_SRCS})
    SET(COMMON_LIBRARIES  ${COMMON_LIBRARIES} ${BULLET_LIBRARIES} SimRobot::SimRobot)
endif()

message("COMMON_LIBRARIES : ", ${COMMON_LIBRARIES})
if(WITH_CASADI)
    add_compile_definitions(__CASADI__)
    find_package(CASADI REQUIRED)
    add_executable(${PROJECT_NAME}
        src/main.cpp
        ${CONTROL_SRCS}
        ${COMMON_SRCS}
    )
    if( Eigen_FOUND )
   add_definitions( ${Eigen_DEFINITIONS} )
   target_include_directories (${PROJECT_NAME} PUBLIC ${Eigen_INCLUDE_DIRS} )
else()
   MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN_INCLUDE_DIR to the include directory of your Eigen installation.")
endif()
    if (CASADI_FOUND)
        MESSAGE(STATUS "CASADI_INCLUDE_DIRS = ${CASADI_INCLUDE_DIR}")
        MESSAGE(STATUS "CASADI_LIBRARIES = ${CASADI_LIBRARIES}")
        target_include_directories(${PROJECT_NAME} PUBLIC 
            ${KERNELHEADERS_DIR}
            ${CASADI_INCLUDE_DIR} 
            ${COMMON_INCLUDE_DIRS}
        )
        target_link_libraries(${PROJECT_NAME} ${CASADI_LIBRARIES})
    endif ()
    target_link_libraries(${PROJECT_NAME}
      ${COMMON_LIBRARIES}
     )
    set( INDY7_CASADI indy7_M indy7_Minv indy7_C indy7_G indy7_id indy7_fd indy7_fk indy7_fk_ee indy7_fkrot_ee indy7_J_s indy7_J_b indy7_J_fd indy7_J_id )
    set( INDYRP_CASADI indyrp2_M indyrp2_Minv indyrp2_C indyrp2_G indyrp2_id indyrp2_fd indyrp2_fk indyrp2_fk_ee indyrp2_fkrot_ee indyrp2_J_s indyrp2_J_b indyrp2_J_fd indyrp2_J_id )
    link_directories(
      "${CMAKE_CURRENT_SOURCE_DIR}/lib/URDF2CASADI/indy7"
      "${CMAKE_CURRENT_SOURCE_DIR}/lib/URDF2CASADI/indyRP"
    )     
endif()

if(WITH_LR)
    add_compile_definitions(__LR__)
    find_package(ruckig REQUIRED)
    find_package(LR REQUIRED)
    find_package(SimRobot REQUIRED)
    find_package(pinocchio REQUIRED)
    add_executable(${PROJECT_NAME}
        src/main.cpp
        ${LR_CONTROL_SRCS}
        ${COMMON_SRCS}
        ${FT_SRCS}
    )
    target_include_directories(${PROJECT_NAME} PUBLIC 
        ${KERNELHEADERS_DIR}
        ${COMMON_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    
    target_link_libraries(${PROJECT_NAME}
        ${COMMON_LIBRARIES}
        LR::Control
        LR::Trajectory
        canlib
     )
     

endif()
if(WITH_BULLET)
    target_compile_definitions(${PROJECT_NAME} PUBLIC -DBT_USE_DOUBLE_PRECISION )
endif()
# ControlBox
if(WITH_RP)
    add_compile_definitions(__RP__)
endif()

# ControlBox
if(WITH_CB)
    add_compile_definitions(__CB__)
endif()

# LieGroupRobotics
if(WITH_LR)
    add_compile_definitions(__LR__)
endif()

# ControlBox
if(WITH_DUALARM)
    add_compile_definitions(__DUALARM__)
endif()

message(STATUS "Xenomai found with theses skins: ${Xenomai_SKINS}")
if(Xenomai_FOUND)
    target_include_directories(${PROJECT_NAME} PUBLIC ${KERNELHEADERS_DIR})
    foreach(comp_lists ${Xenomai_FOUND_COMP})
        target_include_directories(${PROJECT_NAME} PUBLIC ${Xenomai_${comp_lists}_INCLUDE_DIRS})
        target_link_libraries(${PROJECT_NAME} ${Xenomai_${comp_lists}_DL_FLAGS} ${Xenomai_${comp_lists}_LIBRARIES})
        set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS ${Xenomai_${comp_lists}_DEFINITIONS})
    endforeach(comp_lists ${Xenomai_SKINS})
endif(Xenomai_FOUND)
