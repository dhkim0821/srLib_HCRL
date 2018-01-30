FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library (srExt SHARED ${headers} ${sources})
target_link_libraries(srExt srDyn)
