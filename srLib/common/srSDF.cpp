#include "srSDF.h"


using namespace std;
using namespace tinyxml2;
using namespace Eigen;

void srSDF::sdf2srSystem(srSystem * _system, string _stlPath, string _sdfPath)
{
	XMLDocument doc;
	if (doc.LoadFile(_sdfPath.c_str()) != XMLError::XML_SUCCESS)
	{
		cout << "failed to read xml file" << endl;
		return;
	}
	XMLElement* model = doc.FirstChildElement()->FirstChildElement();

	// set links
	for (XMLElement* plink = model->FirstChildElement("link"); plink != NULL; plink = plink->NextSiblingElement("link"))
	{
		//cout << plink->Attribute("name") << endl;
		setSingleLink(_system, plink, _stlPath);
	}

	// set joints
	for (XMLElement* pjoint = model->FirstChildElement("joint"); pjoint != NULL; pjoint = pjoint->NextSiblingElement("joint"))
	{
		//cout << pjoint->Attribute("name") << endl;
		setSingleJoint(_system, pjoint);
	}

	// set base link
	srLink* base = (srLink*)_system->FindObject((char*)model->FirstChildElement("link")->Attribute("name"));
	base->SetFrame(SE3());
	_system->SetBaseLink(base);
	_system->SetBaseLinkType(srSystem::FIXED);
	_system->SetSelfCollision(false);
}

void srSDF::setSingleLink(srSystem * _system, tinyxml2::XMLElement * _plink, std::string _stlPath)
{
	srLink* link = new srLink();
	link->SetName(_plink->Attribute("name"));
	srCollision* coll = new srCollision();
	coll->SetName(_plink->FirstChildElement("collision")->Attribute("name"));


	XMLElement* ele;
	
	// initial frame of link in global frame
	VectorXd pose = string2vec(_plink->FirstChildElement("pose")->GetText(), 6);
	link->m_InitFrame = EulerZYX(Vec3(pose(5), pose(4), pose(3)), Vec3(pose(0), pose(1), pose(2)));
	//cout << link->m_InitFrame << endl;

	// set inertia
	vector<string> angMomentList = { "ixx", "iyy", "izz", "ixy", "iyz", "ixz" };
	if ((ele = _plink->FirstChildElement("inertial")) != NULL)
	{
		VectorXd offset = string2vec(ele->FirstChildElement("pose")->GetText(), 6);
		double mass = atof(ele->FirstChildElement("mass")->GetText());
		VectorXd angMoment(6);
		for (unsigned int i = 0; i < angMomentList.size(); i++)
			angMoment(i) = atof(ele->FirstChildElement("inertia")->FirstChildElement(angMomentList[i].c_str())->GetText());

		link->SetInertia(Inertia(angMoment(0), angMoment(1), angMoment(2), angMoment(3), angMoment(4), angMoment(5),
			offset(0), offset(1), offset(2), mass));
		//link->SetInertia(Inertia(1, 1, 1, 1));
	}

	// set collision
	if ((ele = _plink->FirstChildElement("collision")) != NULL)
	{
		VectorXd pose = string2vec(ele->FirstChildElement("pose")->GetText(), 6);
		string stlName = ele->FirstChildElement("geometry")->FirstChildElement("mesh")->FirstChildElement("uri")->GetText();
		stlName = stlName.substr(stlName.find_last_of("/") + 1);
		//coll = srSTL::STL2Collsion(srSTL::readSTLfile(_stlPath + stlName), "BOX");
		if (coll != NULL)
		{
			coll->GetGeomInfo().SetColor(1.0f, 0.0f, 0.0f, 0.1f);
			//coll->SetPosition()
			coll->GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(pose(5), pose(4), pose(3)), Vec3(pose(0), pose(1), pose(2))));
			link->AddCollision(coll);
		}
	}

	// set visual
	if ((ele = _plink->FirstChildElement("visual")) != NULL)
	{
		VectorXd pose = string2vec(ele->FirstChildElement("pose")->GetText(), 6);
		string stlName = ele->FirstChildElement("geometry")->FirstChildElement("mesh")->FirstChildElement("uri")->GetText();
		stlName = stlName.substr(stlName.find_last_of("/") + 1);
		//cout << stlName << endl;
		if (stlName.substr(stlName.find_last_of(".") + 1) == "STL")
			link->GetGeomInfo().SetShape(srGeometryInfo::STL);
		else
			cout << "not implemented except for '.stl'" << endl;
		//cout << stlName << endl;
		//(_stlPath + stlName).c_str();
		link->GetGeomInfo().SetFileName((char * )(_stlPath + stlName).c_str());
		link->GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(pose(5), pose(4), pose(3)), Vec3(pose(0), pose(1), pose(2))));
	}

}

void srSDF::setSingleJoint(srSystem * _system, tinyxml2::XMLElement * _pjoint)
{
	srLink* plink; // parent link
	srLink* clink; // child link

	XMLElement* ele;

	if (strcmp(_pjoint->Attribute("type"), "revolute") == 0)
	{
		//srRevoluteJoint* joint = new srRevoluteJoint();
		srExtRevoluteJoint* joint = new srExtRevoluteJoint();
		joint->SetName(_pjoint->Attribute("name"));

		plink = (srLink *)_system->FindObject((char *)_pjoint->FirstChildElement("parent")->GetText());
		clink = (srLink *)_system->FindObject((char *)_pjoint->FirstChildElement("child")->GetText());
		joint->SetParentLink(plink);
		joint->SetChildLink(clink);
		VectorXd axis = string2vec(_pjoint->FirstChildElement("axis")->FirstChildElement("xyz")->GetText(), 3);
		VectorXd oth1(3), oth2(3);
		oth1 << 0.0, -axis(2), axis(1);		
		if (oth1.isZero())
			oth1 << -axis(2), 0.0, axis(0);
		oth2 << axis(1) * oth1(2) - axis(2) * oth1(1), axis(2) * oth1(0) - axis(0) * oth1(2), axis(0) * oth1(1) - axis(1) * oth1(0);
		SE3 jointSE3 = SE3(Vec3(oth1(0), oth1(1), oth1(2)), Vec3(oth2(0), oth2(1), oth2(2)), Vec3(axis(0), axis(1), axis(2)), clink->m_InitFrame.GetPosition());
		joint->SetParentLinkFrame(plink->m_InitFrame % jointSE3);
		joint->SetChildLinkFrame(clink->m_InitFrame % jointSE3);

		joint->SetActType(srJoint::ACTTYPE::HYBRID);

		// set limits
		ele = _pjoint->FirstChildElement("axis")->FirstChildElement("limit");
		if ((ele->FirstChildElement("lower") != NULL) && (ele->FirstChildElement("upper") != NULL))
		{
			joint->MakePositionLimit(true);
			joint->SetPositionLimit(RAD2DEG(atof(ele->FirstChildElement("lower")->GetText())), RAD2DEG(atof(ele->FirstChildElement("upper")->GetText())));
		}
		if (ele->FirstChildElement("effort") != NULL)
		{
			joint->SetTorqueLimit(-atof(ele->FirstChildElement("effort")->GetText()), atof(ele->FirstChildElement("effort")->GetText()));
		}
		if (ele->FirstChildElement("velocity") != NULL)
		{
			joint->MakeVelocityLimit(true);
			joint->SetVelocityLimit(RAD2DEG(atof(ele->FirstChildElement("velocity")->GetText())));
		}

	}
	else if (strcmp(_pjoint->Attribute("type"), "prismatic") == 0)
	{
		srPrismaticJoint* joint = new srPrismaticJoint();
		joint->SetName(_pjoint->Attribute("name"));

		plink = (srLink *)_system->FindObject((char *)_pjoint->FirstChildElement("parent")->GetText());
		clink = (srLink *)_system->FindObject((char *)_pjoint->FirstChildElement("child")->GetText());
		joint->SetParentLink(plink);
		joint->SetChildLink(clink);
		VectorXd axis = string2vec(_pjoint->FirstChildElement("axis")->FirstChildElement("xyz")->GetText(), 3);
		VectorXd oth1(3), oth2(3);
		oth1 << 0.0, -axis(2), axis(1);
		if (oth1.isZero())
			oth1 << -axis(2), 0.0, axis(0);
		oth2 << axis(1) * oth1(2) - axis(2) * oth1(1), axis(2) * oth1(0) - axis(0) * oth1(2), axis(0) * oth1(1) - axis(1) * oth1(0);
		SE3 jointSE3 = SE3(Vec3(oth1(0), oth1(1), oth1(2)), Vec3(oth2(0), oth2(1), oth2(2)), Vec3(axis(0), axis(1), axis(2)), clink->m_InitFrame.GetPosition());
		joint->SetParentLinkFrame(plink->m_InitFrame % jointSE3);
		joint->SetChildLinkFrame(clink->m_InitFrame % jointSE3);

		joint->SetActType(srJoint::ACTTYPE::HYBRID);

		ele = _pjoint->FirstChildElement("axis")->FirstChildElement("limit");
		if ((ele->FirstChildElement("lower") != NULL) && (ele->FirstChildElement("upper") != NULL))
		{
			joint->MakePositionLimit(true);
			joint->SetPositionLimit(RAD2DEG(atof(ele->FirstChildElement("lower")->GetText())), RAD2DEG(atof(ele->FirstChildElement("upper")->GetText())));
		}
		if (ele->FirstChildElement("effort") != NULL)
		{
			joint->SetTorqueLimit(-atof(ele->FirstChildElement("effort")->GetText()), atof(ele->FirstChildElement("effort")->GetText()));
		}
		//if (ele->FirstChildElement("velocity") != NULL)
		//{
		//}

	}
	else if (strcmp(_pjoint->Attribute("type"), "fixed") == 0)
	{
		srWeldJoint* joint = new srWeldJoint();
		joint->SetName(_pjoint->Attribute("name"));

		plink = (srLink *)_system->FindObject((char *)_pjoint->FirstChildElement("parent")->GetText());
		clink = (srLink *)_system->FindObject((char *)_pjoint->FirstChildElement("child")->GetText());
		joint->SetParentLink(plink);
		joint->SetChildLink(clink);
		joint->SetParentLinkFrame(plink->m_InitFrame % clink->m_InitFrame);
		joint->SetChildLinkFrame(SE3());

	}
	else
	{
		cout << "not implemented except for 'revolute', 'prismatic', and 'fixed' joint'" << endl;
	}


}

Eigen::VectorXd srSDF::string2vec(std::string _string, int _dim)
{
	VectorXd ret(_dim);
	std::stringstream ss(_string);
	for (int i = 0; i < _dim; i++)
	{
		ss >> ret(i);
	}
	
	return ret;
}


// getTExt() -> 6차원 벡터, 3차원 벡터로 바꾸는 함수..
