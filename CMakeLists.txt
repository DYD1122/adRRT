cmake_minimum_required(VERSION 3.16)
project(adRRT LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/*.cpp")
add_executable(adRRT ${SOURCES})

target_include_directories(adRRT PRIVATE 
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/cxxopts
)

target_link_libraries(adRRT
    PRIVATE
        flann
        lz4
)


