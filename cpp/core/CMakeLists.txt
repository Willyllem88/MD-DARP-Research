add_library(darpmd_core
    CPLEXSolver.cpp
    CPLEXSoftSolver.cpp
    ALNSSolver.cpp
    DARPMD_ProblemInstance.cpp
    DARPMD_ResultInstance.cpp
    alns/ALNSEvaluator.cpp
    alns/SetPartitioningSolver.cpp
    alns/SetCoveringSolver.cpp
    alns/ALNSOperators.cpp
)

target_include_directories(darpmd_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CPLEX_INCLUDE_DIR}
    ${CONCERT_INCLUDE_DIR}
)

target_link_directories(darpmd_core PUBLIC
    ${CPLEX_LIB_DIR}
    ${CONCERT_LIB_DIR}
)

target_link_libraries(darpmd_core PUBLIC
    ilocplex cplex concert pthread m dl
)

add_executable(darpmd_run main.cpp)
target_link_libraries(darpmd_run PRIVATE darpmd_core)
