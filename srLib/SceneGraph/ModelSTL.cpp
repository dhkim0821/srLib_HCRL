#include "ModelSTL.h"

void ModelSTL::Load(string filename)
{
    //list<srSTL::facet> test;
    srSTL::STLc test;
    test.clear();
    std::cout<<test.size()<<std::endl;
    printf("test\n");
    srSTL::facet tmp;
    m_stl_content.push_back(tmp);
    //test.push_back(tmp);
    printf("test2\n");
    m_stl_content.clear();
    printf("here!\n");
    //stl_content = srSTL::readSTLfile(filename);
    srSTL::readSTLfile(filename, m_stl_content);
    printf("stl load 2\n");
    //Draw();
    printf("done\n");
}

void ModelSTL::Draw()
{
    //glPushMatrix();
    //glMultMatrixf(_T);
	for (srSTL::STLc::iterator point = m_stl_content.begin(); point != m_stl_content.end(); point++)
	{
		glBegin(GL_TRIANGLES);
		glNormal3f(point->normal[0], point->normal[1], point->normal[2]);
		for (int i = 0; i < 3; i++)
		{
			glVertex3f(point->vertex[i][0], point->vertex[i][1], point->vertex[i][2]);
		}
		glEnd();
	}
    //glPopMatrix();
}
