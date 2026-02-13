cmake_minimum_required(VERSION 3.16)

project(DARPMD
    VERSION 1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O3 -g -Wall -Wextra")

add_subdirectory(cpp)
