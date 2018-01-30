add_executable (MobileRobots MobileRobots.h MobileRobots.cpp Ground.h)

target_link_libraries (MobileRobots
common
LieGroup
srDyn
srg
#Renderer
SceneGraph 
)
