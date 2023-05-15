#############################################################
# Minimum required version                                  #
#############################################################
cmake_minimum_required(VERSION 3.1)
include(CMakeDependentOption)

#############################################################
# Install & Deployment Setting                              #
#############################################################
set(DEPLOYMENT_DIR "bin")

#############################################################
# Toolchains                                                #
#############################################################
set(CMAKE_CROSSCOMPILING true)
set(CMAKE_HOST_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_NAME Linux)
set(NRMK_PLATFORM "STEP2")

set(ENV{CC} "${CMAKE_CURRENT_SOURCE_DIR}/toolchain/i686-unknown-linux-gnu/bin/i686-unknown-linux-gnu-gcc")
set(ENV{CXX} "${CMAKE_CURRENT_SOURCE_DIR}/toolchain/i686-unknown-linux-gnu/bin/i686-unknown-linux-gnu-g++")
set(CMAKE_SYSTEM_PROCESSOR i686)


#############################################################
# Project name                                              #
#############################################################
project(RTIndy7_01)

#############################################################
# Project Setting                                           #
#############################################################
set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/${DEPLOYMENT_DIR}" CACHE PATH "Install path prefix, prepended onto install directories." FORCE)
set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)

#############################################################
# CPP Compiler & Linker flags                               #
#############################################################
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_C_COMPILER_VERSION  6.5)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -m32 -lrt -fmessage-length=0 -ffast-math")

# Release flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
# Debug flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -O0")

#############################################################
# Definitions                                               #
#############################################################
add_definitions(-D__XENO__)
set(ROBOT_TYPE AbstractRobot6D)
add_definitions(-D__ROBOT_6DOF__)
#############################################################
# Includes                                                  #
#############################################################
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/NRMKSDK/xenomai")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/NRMKSDK/ethercat")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/NRMKSDK/core")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/NRMKSDK/helper")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/NRMKSDK/helper/hw")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/NRMKSDK/helper/hw/nrmk_ecat")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/NRMKSDK/eigen")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/NRMKSDK/Poco")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/NRMKSDK/json")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/NRMKSDK/modbus")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/NRMKSDK/qpOASES")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/NRMKSDK/cryptopp")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/NRMKSDK/NRMKFramework")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/NRMKSDK/ModernRobotics")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/KDL")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/CAN")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/EcatSystem")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/Control")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/MR")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}/include/EcatDataSocket")
include_directories(	"${CMAKE_CURRENT_SOURCE_DIR}")
message("Done")

#############################################################
# Libraries                                                 #
#############################################################
# Define LIBS

set( LIBS
	xenomai
	native
	rt
	rtdm
	ethercat_rtdm
	pthread
	NRMKFramework
	NRMKCore
	NRMKHelperi686
	NRMKhw_tp
	NRMKhw_tpd
	PocoUtil
	PocoXML
	PocoJSON
	PocoNet
	PocoFoundation
	z
	jsoncpp
	hiredis
	microhttpd
	curl
	ssl
	crypto
	argtable2
	jsonrpccpp-common
	jsonrpccpp-stub
	jsonrpccpp-server
	jsonrpccpp-client
	modbus
	qpOASES
	cryptopp
	NRMKFramework
)

link_directories(
	"${CMAKE_CURRENT_SOURCE_DIR}/lib/xenomai"
	"${CMAKE_CURRENT_SOURCE_DIR}/lib/ethercat"
	"${CMAKE_CURRENT_SOURCE_DIR}/lib/lib"
	"${CMAKE_CURRENT_SOURCE_DIR}/lib/Poco"
	"${CMAKE_CURRENT_SOURCE_DIR}/lib/json"
	"${CMAKE_CURRENT_SOURCE_DIR}/lib/modbus"	
	"${CMAKE_CURRENT_SOURCE_DIR}/lib/qpOASES"		
	"${CMAKE_CURRENT_SOURCE_DIR}/lib/cryptopp"
	"${CMAKE_CURRENT_SOURCE_DIR}/lib/NRMKFramework"
	
)

#############################################################
# Sources                                                   #
#############################################################
file(GLOB_RECURSE MR include/MR/modern_robotics.h include/MR/modern_robotics.cpp)
file(GLOB_RECURSE MR_Indy7 include/MR/MR_Indy7.h include/MR/MR_Indy7.cpp)
file(GLOB_RECURSE KDL  "include/KDL/*.h" "include/KDL/*.cpp")
file(GLOB_RECURSE PDO include/EcatSystem/PDOConfig.c include/EcatSystem/PDOConfig.h)
file(GLOB_RECURSE EcatSystem include/EcatSystem/Ecat_Master.cpp include/EcatSystem/Ecat_Master.h)
file(GLOB_RECURSE Control include/Control/*.h include/Control/*.cpp include/Control/*.c)
file(GLOB_RECURSE EcatDataSocket include/EcatDataSocket/EcatDataSocket.cpp include/EcatDataSocket/EcatDataSocket.h include/EcatDataSocket/EcatControlSocket.h) 
file(GLOB_RECURSE CAN include/CAN/*.h include/CAN/*.cpp include/CAN/*.c)
file(GLOB_RECURSE SOURCES "src/RTIndy7Client.cpp")



#############################################################
# Executable                                                #
#############################################################
set(EXE ${PROJECT_NAME})
add_executable(${EXE} ${SOURCES} ${PDO} ${MR_Indy7} ${MR} ${EcatDataSocket}   ${EcatSystem} ${CAN} )
set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX  "")

#############################################################
# Libraries linking                                         #
#############################################################
target_link_libraries(${EXE} ${LIBS} )

