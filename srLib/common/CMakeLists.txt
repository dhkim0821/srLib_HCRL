FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)

add_library (common ${headers} ${sources})
#add_library (common SHARED ${headers} ${sources})

target_link_libraries(common tinyxml2)
target_link_libraries(common srDyn)
