cmake_minimum_required(VERSION 3.16)

project(DARPMD
    VERSION 1.0
    DESCRIPTION "Dial-a-Ride Problem with Multi-Depots Solver"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "-O3 -g -Wall -Wextra")

## CPLEX configuration
if(NOT DEFINED CPLEX_ROOT)
    set(CPLEX_ROOT "/opt/ibm/ILOG/CPLEX_Studio2211")
endif()

set(CPLEX_INCLUDE_DIR "${CPLEX_ROOT}/cplex/include")
set(CONCERT_INCLUDE_DIR "${CPLEX_ROOT}/concert/include")
set(CPLEX_LIB_DIR "${CPLEX_ROOT}/cplex/lib/x86-64_linux/static_pic")
set(CONCERT_LIB_DIR "${CPLEX_ROOT}/concert/lib/x86-64_linux/static_pic")

add_custom_target(install_cordeau_instances
    COMMAND bash ${CMAKE_SOURCE_DIR}/scripts/install_cordeau_instances.sh
    COMMENT "Downloading and converting Cordeau instances (may take a while)..."
)
add_subdirectory(cpp)
