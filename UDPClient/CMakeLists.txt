cmake_minimum_required(VERSION 3.5)
project(udp_server_client)

set(CMAKE_CXX_STANDARD 11)

# Poco 라이브러리 경로 설정
set(POCO_PREFIX "/usr/local")
set(POCO_INCLUDE_DIR "${POCO_PREFIX}/include")
set(POCO_LIB_DIR "${POCO_PREFIX}/lib")

# Poco 라이브러리 링크
find_library(POCO_FOUNDATION PocoFoundation PATHS "${POCO_LIB_DIR}")
find_library(POCO_NET PocoNet PATHS "${POCO_LIB_DIR}")
find_library(POCO_JSON PocoJSON PATHS "${POCO_LIB_DIR}")  # 추가

# 빌드 대상 파일 지정
add_executable(udp_server udp_server.cpp)
add_executable(udp_client udp_client.cpp)

# 라이브러리 링크
target_link_libraries(udp_server ${POCO_FOUNDATION} ${POCO_NET} ${POCO_JSON})
target_link_libraries(udp_client ${POCO_FOUNDATION} ${POCO_NET} ${POCO_JSON})
