add_library(mddarp_core
    CPLEXSolver.cpp
    CPLEXSoftSolver.cpp
    ALNSSolver.cpp
    MDDARP_ProblemInstance.cpp
    MDDARP_ResultInstance.cpp
    alns/ALNSEvaluator.cpp
    alns/SetPartitioningSolver.cpp
    alns/SetCoveringSolver.cpp
    alns/ALNSOperators.cpp
    alns/RoutePool.cpp
)

target_include_directories(mddarp_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CPLEX_INCLUDE_DIR}
    ${CONCERT_INCLUDE_DIR}
)

target_link_directories(mddarp_core PUBLIC
    ${CPLEX_LIB_DIR}
    ${CONCERT_LIB_DIR}
)

target_link_libraries(mddarp_core PUBLIC
    ilocplex cplex concert pthread m dl
)

add_executable(mddarp_run main.cpp)
target_link_libraries(mddarp_run PRIVATE mddarp_core)
