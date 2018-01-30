FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library (LieGroup ${headers} ${sources})
#add_library (LieGroup SHARED ${headers} ${sources})

target_link_libraries(LieGroup common)

