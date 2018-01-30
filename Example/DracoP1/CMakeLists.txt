FILE(GLOB_RECURSE header *.h)
file(GLOB_RECURSE source *.cpp)
add_executable (simulation_draco ${header} ${source} )

target_link_libraries (simulation_draco
  common
  LieGroup
  srDyn
  srg
  util
  ${OPENGL_LIBRARIES}
  SceneGraph
)

if (APPLE)
  target_link_libraries(simulation_draco "-framework GLUT")
endif (APPLE)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
  target_link_libraries(simulation_draco ${GLUT_glut_LIBRARIES})
  include (CheckIncludeFileCXX)
endif (CMAKE_SYSTEM_NAME MATCHES Linux)
