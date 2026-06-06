cmake_minimum_required(VERSION 3.16)

project(MDDARP
    VERSION 1.0
    DESCRIPTION "Multi-Depot Dial-a-Ride Problem Solver"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-O3 -Wall -Wextra)

## CPLEX configuration
set(
    CPLEX_ROOT
    "$ENV{CPLEX_ROOT}"
    CACHE PATH "Path to the CPLEX Studio installation"
)
if(NOT CPLEX_ROOT)
    set(CPLEX_ROOT "/opt/ibm/ILOG/CPLEX_Studio2211")
endif()

set(CPLEX_INCLUDE_DIR "${CPLEX_ROOT}/cplex/include")
set(CONCERT_INCLUDE_DIR "${CPLEX_ROOT}/concert/include")
set(CPLEX_LIB_DIR "${CPLEX_ROOT}/cplex/lib/x86-64_linux/static_pic")
set(CONCERT_LIB_DIR "${CPLEX_ROOT}/concert/lib/x86-64_linux/static_pic")

add_custom_target(install_cordeau_instances
    COMMAND bash ${CMAKE_SOURCE_DIR}/scripts/install_cordeau_instances.sh
    COMMAND bash ${CMAKE_SOURCE_DIR}/scripts/install_cordeau_instances_tabu.sh
    COMMENT "Downloading and converting Cordeau instances (may take a while)..."
)
add_subdirectory(cpp)
