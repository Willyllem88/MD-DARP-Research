# Executable definition
add_executable(instance_analyzer
    main_analyzer.cpp
    InstanceAnalyzer.cpp
)

target_link_libraries(instance_analyzer PRIVATE darpmd_core)