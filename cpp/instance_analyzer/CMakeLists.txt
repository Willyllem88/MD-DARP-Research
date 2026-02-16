# Executable definition
add_executable(instance_analyzer
    main_analyzer.cpp
    InstanceAnalyzer.cpp
)

# Link against the core library (Solver, ProblemInstance, etc.)
# This inherits all include paths (json.hpp, CPLEX) automatically.
target_link_libraries(instance_analyzer PRIVATE darpmd_core)