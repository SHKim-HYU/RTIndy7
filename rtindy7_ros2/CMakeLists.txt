cmake_minimum_required(VERSION 3.8)
project(rtindy7_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)
find_library(POCO_FOUNDATION PocoFoundation PATHS "${POCO_LIB_DIR}")
find_library(POCO_NET PocoNet PATHS "${POCO_LIB_DIR}")
find_library(POCO_JSON PocoJSON PATHS "${POCO_LIB_DIR}")  # 추가


set(POCO_PREFIX "/usr/local")
set(POCO_INCLUDE_DIR "${POCO_PREFIX}/include")
set(POCO_LIB_DIR "${POCO_PREFIX}/lib")

file(GLOB_RECURSE MR "include/ModernRobotics/*.cpp")
file(GLOB_RECURSE MR2 "include/ModernRobotics_origin/*.cpp")
file(GLOB_RECURSE INDY7 "include/Indy7/*.cpp")
add_executable(mr_joint_space_control src/mr_joint_space_control.cpp  ${INDY7} ${MR} ${MR2})
add_executable(client src/client.cpp )
target_link_libraries(client ${POCO_FOUNDATION} ${POCO_NET} ${POCO_JSON})
target_include_directories(mr_joint_space_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ModernRobotics>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ModernRobotics_origin>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/Indy7>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/json>
  $<INSTALL_INTERFACE:include>)
  target_link_libraries(mr_joint_space_control jsoncpp_lib)
  
  target_compile_features(mr_joint_space_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  mr_joint_space_control
  "robot_state_publisher"
  "sensor_msgs"  
  "jsoncpp"
)
ament_target_dependencies(
  client
  "robot_state_publisher"
  "sensor_msgs"  
)
add_custom_command(TARGET mr_joint_space_control POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/MR_info.json lib/${PROJECT_NAME})
add_custom_target(copy_MR_info ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/MR_info.json)


install(TARGETS mr_joint_space_control
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS client
  DESTINATION lib/${PROJECT_NAME})
install(
    DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/urdf/
    DESTINATION share/${PROJECT_NAME}
)
install(
    DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/meshes/visual/
    DESTINATION share/${PROJECT_NAME}/meshes/visual/
)
install(
    DIRECTORY  ${CMAKE_CURRENT_SOURCE_DIR}/launch/
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
