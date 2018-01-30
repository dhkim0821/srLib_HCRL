FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library (srDyn ${headers} ${sources})
#add_library (srDyn SHARED ${headers} ${sources})

target_link_libraries(srDyn common)
target_link_libraries(srDyn LieGroup)
