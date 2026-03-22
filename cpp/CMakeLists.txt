# We make that 'includes' is visible to all subdirectories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/includes)

add_subdirectory(core)
add_subdirectory(tests)
add_subdirectory(instance_analyzer)
