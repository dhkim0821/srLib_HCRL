FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)

add_library (srg SHARED ${headers} ${sources})
target_link_libraries(srg common)
target_link_libraries(srg LieGroup)
target_link_libraries(srg ${OPENGL_LIBRARIES})
target_link_libraries(srg ${GLUT_glut_LIBRARY})
target_link_libraries(srg SceneGraph)
