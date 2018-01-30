################################################
################################################
cmake_minimum_required(VERSION 3.0)
project(SDC_Toolkit)

set(CMAKE_CXX_FLAGS "-std=c++1z")

set (CMAKE_BUILD_TYPE Debug)
set (CMAKE_BUILD_TYPE Release)
add_definitions(-O3)
add_definitions(-D_MAX_PATH=20)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIR})

find_package(OpenSceneGraph 2.0.0 REQUIRED osgDB osgUtil)
include_directories(${OSG_INCLUDE_DIR})

find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})

include_directories("${PROJECT_SOURCE_DIR}/Eigen")
include_directories("${PROJECT_SOURCE_DIR}/srLib")
include_directories("${PROJECT_SOURCE_DIR}")
#------------------------------------------------------------------------
#    Directory Selection
#------------------------------------------------------------------------
#add_subdirectory (Eigen)
#add_subdirectory (Example/MobileRobots)
add_subdirectory(Example/DracoP1)
add_subdirectory (srLib)
add_subdirectory (srMath)
add_subdirectory (tinyxml2)

#------------------------------------------------------------------------
message ("")
message ("-[ MAKE INFORMATION ]----------------------------------------")
message (" The project name: ${PROJECT_NAME}")
message (" The version: ${${PROJECT_NAME}_VERSION}")
message (" The project source directory: ${PROJECT_SOURCE_DIR}")
message (" The project binary directory: ${PROJECT_BINARY_DIR}")
message (" The install destination: ${CMAKE_INSTALL_PREFIX}")
message (" The control system: ${CTRL_SYS}")
message ("--------------------------------------------------------------")
message ("")


