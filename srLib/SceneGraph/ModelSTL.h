#ifndef __MODELSTL__
#define __MODELSTL__

#include <stdlib.h>
#include <glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include "common/srSTL.h"

class ModelSTL
{
private:
    srSTL::STLc m_stl_content;
    //list<srSTL::facet> stl_content;

public:
    ModelSTL(){ m_stl_content.clear(); }
    ModelSTL(string name){
    Load(name);
    }
	float _T[16];

	void Load(string);
	void Draw();

    //srSTL::STLc m_stl_content;
};

#endif
