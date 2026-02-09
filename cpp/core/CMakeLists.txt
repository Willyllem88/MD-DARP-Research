# Ejecutable principal
add_executable(darpmd
    main.cpp
    CPLEXSolver.cpp
    CPLEXSoftSolver.cpp
    ALNSSolver.cpp
    DARPMD_ProblemInstance.cpp
    DARPMD_ResultInstance.cpp
)

# Includes del proyecto
target_include_directories(darpmd PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    /opt/ibm/ILOG/CPLEX_Studio2211/cplex/include
    /opt/ibm/ILOG/CPLEX_Studio2211/concert/include
)

# Librerías CPLEX (ajusta arquitectura si hace falta)
target_link_directories(darpmd PRIVATE
    /opt/ibm/ILOG/CPLEX_Studio2211/cplex/lib/x86-64_linux/static_pic
    /opt/ibm/ILOG/CPLEX_Studio2211/concert/lib/x86-64_linux/static_pic
)

target_link_libraries(darpmd
    ilocplex
    cplex
    concert
    pthread
    m
)
