cmake_minimum_required(VERSION 3.18)
project(AR1Linear)

set(CMAKE_CXX_STANDARD 17)

file(GLOB_RECURSE SRC_FILES
        src/*.cpp
        src/*.h
        src/*.hpp
)

add_library(AR1Linear STATIC ${SRC_FILES})
target_include_directories(AR1Linear PUBLIC src)