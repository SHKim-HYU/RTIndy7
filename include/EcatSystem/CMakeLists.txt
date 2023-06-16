cmake_minimum_required(VERSION 3.8)
project(EcatSystem LANGUAGES C CXX)

set(CMAKE_DEBUG_POSTFIX "d")
file(GLOB SOURCE_FILES "*.cpp" "*.c")

add_library(${PROJECT_NAME} SHARED STATIC
            ${SOURCE_FILES}
            )

add_library(sub::ECAT ALIAS ${PROJECT_NAME})

target_include_directories( ${PROJECT_NAME}
        PUBLIC ${PRJECT_SOURCE_DIR})

find_package(Etherlab REQUIRED)
if(Etherlab_FOUND)
    message(STATUS "Etherlab_INC_DIRS = ${Etherlab_INCLUDE_DIR}")
    message(STATUS "Etherlab_LIB_DIRS = ${Etherlab_LIBRARIES}")
    target_include_directories(${PROJECT_NAME} PUBLIC ${Etherlab_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} PUBLIC ${Etherlab_LIBRARIES})
endif(Etherlab_FOUND)

find_package(Xenomai 3.1.1 REQUIRED NATIVE)
if(Xenomai_FOUND)
    target_include_directories(${PROJECT_NAME} PUBLIC ${KERNELHEADERS_DIR})
    foreach(comp_lists ${Xenomai_FOUND_COMP})
        #message(STATUS "Xenomai ${comp_lists} includes : ${Xenomai_${comp_lists}_INCLUDE_DIRS}" )
        #message(STATUS "Xenomai ${comp_lists} definitions: ${Xenomai_${comp_lists}_DEFINITIONS}")
        #message(STATUS "Xenomai ${comp_lists} LIBRARY_DIRS : ${Xenomai_${comp_lists}_LIBRARY_DIRS} ")
        #message(STATUS "Xenomai ${comp_lists} LIBRARIES : ${Xenomai_${comp_lists}_LIBRARIES}")
        #message(STATUS "Xenomai ${comp_lists} LDFLAGS : ${Xenomai_${comp_lists}_LDFLAGS}")
        #message(STATUS "Xenomai ${comp_lists} DL_FLAGS : ${Xenomai_${comp_lists}_DL_FLAGS}")

        target_include_directories(${PROJECT_NAME} PUBLIC ${Xenomai_${comp_lists}_INCLUDE_DIRS})
        #target_link_libraries(${PROJECT_NAME} ${Xenomai_${comp_lists}_LIBRARY_DIRS} ${Xenomai_${comp_lists}_LIBRARIES})
        target_link_libraries(${PROJECT_NAME} PUBLIC ${Xenomai_${comp_lists}_LIBRARIES})
        set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS ${Xenomai_${comp_lists}_DEFINITIONS})
    endforeach(comp_lists ${Xenomai_SKINS})
endif(Xenomai_FOUND)
if(Xenomai_NOT_FOUND)
	message("MCMCMCMC")
endif(Xenomai_NOT_FOUND)
install(TARGETS ${PROJECT_MANE}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
