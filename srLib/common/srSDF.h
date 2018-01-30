#ifndef __SR_SDF__
#define __SR_SDF__


#include <srDyn/srSpace.h>
#include <tinyxml2/tinyxml2.h>
#include <Eigen/Core>
#include <vector>
#include "srSTL.h"
#include <srExt/srExt_RevoluteJoint.h>


namespace srSDF
{
	void sdf2srSystem(srSystem* _system, std::string _stlPath, std::string _sdfPath);

	void setSingleLink(srSystem* _system, tinyxml2::XMLElement* _plink, std::string _stlPath);

	void setSingleJoint(srSystem* _system, tinyxml2::XMLElement* _pjoint);

	Eigen::VectorXd string2vec(std::string _string, int _dim);
}



#endif
