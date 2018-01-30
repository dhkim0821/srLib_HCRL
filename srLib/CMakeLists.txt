find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})

find_package(OpenSceneGraph 2.0.0 REQUIRED osgDB osgUtil)
include_directories(${OSG_INCLUDE_DIR})

message(${OSG_INCLUDE_DIR})
message(${OPENGL_INCLUDE_DIR})
#FIND_PACKAGE(OpenThreads)
#FIND_PACKAGE(osg)
#FIND_PACKAGE(osgDB)
#FIND_PACKAGE(osgUtil)
#FIND_PACKAGE(osgGA)
#FIND_PACKAGE(osgViewer)

if (APPLE)
    include_directories("/opt/X11/include/GL/")
endif (APPLE)

if (CMAKE_SYSTEM_NAME MATCHES Linux)
    find_package(GLUT REQUIRED)
    cmake_policy(SET CMP0003 NEW)
endif (CMAKE_SYSTEM_NAME MATCHES Linux)


add_subdirectory(common)
add_subdirectory(LieGroup)
add_subdirectory(SceneGraph)
add_subdirectory(srDyn)
add_subdirectory(srExt)
add_subdirectory(srg)
#add_subdirectory(Renderer)
