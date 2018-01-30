FILE(GLOB_RECURSE headers *.h)
FILE(GLOB_RECURSE sources *.cpp)

add_library(srMath SHARED ${headers} ${sources})
target_link_libraries(srMath tinyxml2)
