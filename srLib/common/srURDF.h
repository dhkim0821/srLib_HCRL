/*******************************************************************/
/*                                                                 */
/*    FOR USING URDF FORMAT IN SRLIB                               */
/*    MADE BY: Keunjun Choi                                        */
/*    Date: 2014-12-15                                             */
/*                                NAMESPACE:     srURDF            */
/*                                                                 */
/*******************************************************************/

#ifndef __SR_URDF__
#define __SR_URDF__

//#define srLOG

#include <iostream>
//#include </algorithm/string.hpp>
#include <fstream>

#include <urdf_parser/urdf_parser.h>
#include "srRobot.h"
#include "srSTL.h"

#ifndef WIN32
//#include "BasicFiles/BasicSetting.h"
#endif

using namespace std;

namespace srURDF
{
	static string urlcon(string url, string package_url)
	{
		return package_url.append(url.c_str() + 9);
	}

	static void urdf2srRobot(srrobot::srRobot* mRobot, string package_url, string urdfFile, bool collisionflag = true)
	{

#ifdef srLOG
		printf("urdf 파일을 srLib에서 쓸 수 있도록 변환시켜 줍니다.\n파일: %s\n", (package_url + "/robots/" + urdfFile + ".stl").c_str());
#endif

#ifdef srLOG
		printf(".urdf 파일을 읽기 시작합니다.\n");
#endif
		for (int i = package_url.length() - 1; i >= 0; i--)
		{
			if (package_url[i] == '\n' || package_url[i] == '/' || package_url[i] == '\\')
			{
				package_url.erase(i);
				continue;
			}
			break;
		}

		string xml_string;
		fstream xml_file(urdfFile, fstream::in);
		if (!xml_file.good())
		{
#ifdef srLOG
			printf("Cannot open file..\n");
#endif
#ifndef WIN32
        //FILE_LOG(logSUCCESS) << "Cannot open file : "<<urdfFile;
#endif
			return;
		}
		while (xml_file.good())
		{
			std::string line;
			std::getline(xml_file, line);
			xml_string += (line + "\n");
		}
		xml_file.close();
#ifdef srLOG
		printf("파일을 다 읽었습니다.\n");
#endif

#ifdef srLOG
		printf(".urdf 파일을 로봇 형태로 읽어드립니다.");
#endif
		//FILE_LOG(logSUCCESS) << xml_string;
		urdf::ModelInterfaceSharedPtr robot = urdf::parseURDF(xml_string);
		//urdf::ModelInterfaceSharedPtr robot;
		if (!robot)
		{
#ifdef srLOG
			printf(".urdf 파일에 형태가 맞지 않습니다. 확인해주세요.\n");
#endif
			return;
		}

#ifdef srLOG
		printf("로봇을 srRobot으로 변환합니다.\n");
#endif
		urdf::LinkConstSharedPtr root_link = robot->getRoot();
#ifdef srLOG
		printf("Link 정보를 불러옵니다.\n");
#endif
		srLink* link;
		srCollision* coll;
		double r, p, y;
		string fn;
		for (map<std::string, urdf::LinkSharedPtr >::iterator child = robot->links_.begin(); child != robot->links_.end(); child++)
		{
			urdf::LinkSharedPtr linkp = child->second;
#ifdef srLOG
			printf("Link (%s) 가 들어왔습니다.\n", (linkp->name).c_str());
#endif

			link = mRobot->setLink(linkp->name);
			mRobot->getLink(linkp->name)->SetName(linkp->name);
			coll = mRobot->setCollision(linkp->name);

#ifdef srLOG
			printf("Inertia, mass 추가\n");
#endif
			if (linkp->inertial != NULL)
			{
				// TODO: see below
				//link->SetInertia(Inertia(linkp->inertial->mass, linkp->inertial->ixx, linkp->inertial->iyy, linkp->inertial->izz));
				//link->SetInertia(Inertia(linkp->inertial->mass, linkp->inertial->ixx, 1, linkp->inertial->izz));
				link->SetInertia(Inertia(1, 1, 1, 1));
			}

#ifdef srLOG
			printf("Visual 추가\n");
#endif
			if (linkp->visual != NULL)
			{
				switch (linkp->visual->geometry->type)
				{
				case urdf::Geometry::MESH:
					fn = ((urdf::Mesh*)linkp->visual->geometry.get())->filename;
					//boost::to_lower(fn);
					std::transform(fn.begin(), fn.end(), fn.begin(), ::tolower);
					if (fn.substr(fn.find_last_of(".")+1) == "stl")
					{
						link->GetGeomInfo().SetShape(srGeometryInfo::STL);
					}
					else
					{
						link->GetGeomInfo().SetShape(srGeometryInfo::TDS);
					}
					if (linkp->visual->material != NULL)
					{
						link->GetGeomInfo().SetColor(linkp->visual->material->color.r, linkp->visual->material->color.g, linkp->visual->material->color.b, linkp->visual->material->color.a);
					}
					link->GetGeomInfo().SetFileName((char *)urlcon(((urdf::Mesh*)linkp->visual->geometry.get())->filename, package_url).c_str());
					linkp->visual->origin.rotation.getRPY(r, p, y);
					link->GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(y, p, r), Vec3(linkp->visual->origin.position.x, linkp->visual->origin.position.y, linkp->visual->origin.position.z)));
					break;
				default:
#ifdef srLOG
					printf("Visual Geometry Type에 아직 구현되지 않은 정보가 들어왔습니다. 구현해주세요.\n");
#endif
					break;
				}
			}

#ifdef srLOG
			printf("Collision 추가\n");
#endif
			if (linkp->collision != NULL && collisionflag)
			{
				switch (linkp->collision->geometry->type)
				{
				case urdf::Geometry::MESH:
					coll = srSTL::STL2Collsion(srSTL::readSTLfile(urlcon(((urdf::Mesh*)linkp->collision->geometry.get())->filename, package_url)), "BOX");
					if (coll == NULL) break;
					coll->GetGeomInfo().SetColor(1.0f, 0.0f, 0.0f, 0.1f);
					coll->SetPosition(Vec3(linkp->visual->origin.position.x, linkp->visual->origin.position.y, linkp->visual->origin.position.z));
					linkp->visual->origin.rotation.getRPY(r, p, y);
					coll->GetGeomInfo().SetLocalFrame(EulerZYX(Vec3(y, p, r), Vec3(linkp->visual->origin.position.x, linkp->visual->origin.position.y, linkp->visual->origin.position.z)));
					link->AddCollision(coll);
					break;
				default:
#ifdef srLOG
					printf("Collision Geometry Type에 아직 구현되지 않은 정보가 들어왔습니다. 구현해주세요.\n");
#endif
					break;
				}
			}

#ifdef srLOG
			printf("Link 불러오기 성공\n");
#endif
		}

#ifdef srLOG
		printf("Joint 정보를 불러옵니다.\n");
#endif
		srJoint* joint;
		srRevoluteJoint* rjoint;
		Vec3 axis, oth1, oth2;
		SE3 rot;
		for (map<std::string, urdf::JointSharedPtr >::iterator child = robot->joints_.begin(); child != robot->joints_.end(); child++)
		{
			urdf::JointSharedPtr jointp = child->second;
#ifdef srLOG
			printf("Joint (%s) 가 들어왔습니다.\n", (jointp->name).c_str());
#endif

			switch (jointp->type)
			{
			case urdf::Joint::FIXED:
				joint = mRobot->setJoint(jointp->name, srJoint::WELD);
				mRobot->getJoint(jointp->name)->SetName(jointp->name);
				joint->SetChildLink(mRobot->getLink(jointp->child_link_name));
				joint->SetParentLink(mRobot->getLink(jointp->parent_link_name));
				//joint->m_ChildLink->m_ParentLink = joint->m_ParentLink;
				r = p = y = 0;
				jointp->parent_to_joint_origin_transform.rotation.getRPY(r, p, y);
				joint->SetParentLinkFrame(EulerZYX(Vec3(0, 0, 0), Vec3(jointp->parent_to_joint_origin_transform.position.x, jointp->parent_to_joint_origin_transform.position.y, jointp->parent_to_joint_origin_transform.position.z)) * EulerZYX(Vec3(y, p, r), Vec3(0, 0, 0)));
				break;

			case urdf::Joint::REVOLUTE:
				//cout << jointp->limits->lower << '\t' << jointp->limits->upper << endl;
				axis = Vec3(jointp->axis.x, jointp->axis.y, jointp->axis.z);
				oth1 = Vec3(jointp->axis.z, jointp->axis.z, -(jointp->axis.x + jointp->axis.y));
				oth2 = Vec3(-(jointp->axis.y + jointp->axis.z), jointp->axis.x, jointp->axis.x);
				if (oth1 == Vec3(0, 0, 0))
					oth1.Clone(oth2);
				oth1.Normalize();
				oth2 = Cross(axis, oth1);
				rot = SE3(oth1[0], oth2[0], axis[0], oth1[1], oth2[1], axis[1], oth1[2], oth2[2], axis[2]);

				rjoint = (srRevoluteJoint*)mRobot->setJoint(jointp->name, srJoint::REVOLUTE);
				rjoint->SetActType(srJoint::ACTTYPE::HYBRID);
				//joint->SetActType(srJoint::ACTTYPE::TORQUE);

				rjoint->MakePositionLimit(true);
				rjoint->SetPositionLimit(RAD2DEG(jointp->limits->lower), RAD2DEG(jointp->limits->upper));

				rjoint->SetChildLink(mRobot->getLink(jointp->child_link_name));
				rjoint->SetParentLink(mRobot->getLink(jointp->parent_link_name));
				//rjoint->m_ChildLink->m_ParentLink = rjoint->m_ParentLink;
				r = p = y = 0;
				jointp->parent_to_joint_origin_transform.rotation.getRPY(r, p, y);
				rjoint->SetParentLinkFrame(EulerZYX(Vec3(0, 0, 0), Vec3(jointp->parent_to_joint_origin_transform.position.x, jointp->parent_to_joint_origin_transform.position.y, jointp->parent_to_joint_origin_transform.position.z)) * EulerZYX(Vec3(y, p, r)) * Inv(rot));
				rjoint->SetChildLinkFrame(Inv(rot));


				//joint = mRobot->getJoint(jointp->name, srJoint::REVOLUTE);
				//joint->SetActType(srJoint::ACTTYPE::HYBRID);
				////joint->SetActType(srJoint::ACTTYPE::TORQUE);
				//joint->SetChildLink(mRobot->getLink(jointp->child_link_name));
				//joint->SetParentLink(mRobot->getLink(jointp->parent_link_name));
				//if (jointp->child_link_name == "NX03_B03_HINGE-H-T-7-1")
				//{
				//	r = r;
				//}
				//r = p = y = 0;
				//jointp->parent_to_joint_origin_transform.rotation.getRPY(r, p, y);
				//joint->SetParentLinkFrame(EulerZYX(Vec3(0, 0, 0), Vec3(jointp->parent_to_joint_origin_transform.position.x, jointp->parent_to_joint_origin_transform.position.y, jointp->parent_to_joint_origin_transform.position.z)) * EulerZYX(Vec3(y, p, r)) * Inv(rot));
				//joint->SetChildLinkFrame(Inv(rot));
				break;

			case urdf::Joint::CONTINUOUS:
				//cout << jointp->limits->lower << '\t' << jointp->limits->upper << endl;
				axis = Vec3(jointp->axis.x, jointp->axis.y, jointp->axis.z);
				oth1 = Vec3(jointp->axis.z, jointp->axis.z, -(jointp->axis.x + jointp->axis.y));
				oth2 = Vec3(-(jointp->axis.y + jointp->axis.z), jointp->axis.x, jointp->axis.x);
				if (oth1 == Vec3(0, 0, 0))
					oth1.Clone(oth2);
				oth1.Normalize();
				oth2 = Cross(axis, oth1);
				rot = SE3(oth1[0], oth2[0], axis[0], oth1[1], oth2[1], axis[1], oth1[2], oth2[2], axis[2]);

				rjoint = (srRevoluteJoint*)mRobot->setJoint(jointp->name, srJoint::REVOLUTE);
				rjoint->SetActType(srJoint::ACTTYPE::HYBRID);
				//joint->SetActType(srJoint::ACTTYPE::TORQUE);

				rjoint->MakePositionLimit(false);

				rjoint->SetChildLink(mRobot->getLink(jointp->child_link_name));
				rjoint->SetParentLink(mRobot->getLink(jointp->parent_link_name));
				//rjoint->m_ChildLink->m_ParentLink = rjoint->m_ParentLink;
				r = p = y = 0;
				jointp->parent_to_joint_origin_transform.rotation.getRPY(r, p, y);
				rjoint->SetParentLinkFrame(EulerZYX(Vec3(0, 0, 0), Vec3(jointp->parent_to_joint_origin_transform.position.x, jointp->parent_to_joint_origin_transform.position.y, jointp->parent_to_joint_origin_transform.position.z)) * EulerZYX(Vec3(y, p, r)) * Inv(rot));
				rjoint->SetChildLinkFrame(Inv(rot));

				break;


			default:
#ifdef srLOG
				printf("Joint Type에 아직 구현되지 않은 정보가 들어왔습니다. 구현해주세요.\n");
#endif
				break;
			}

#ifdef srLOG
			printf("Joint 불러오기 성공\n");
#endif
		}

#ifdef srLOG
		printf("srLib에서 로봇을 쓸 수 있도록 마무리 작업중입니다.\n");
#endif
		srLink* base = mRobot->getLink(robot->getRoot()->name);
		base->SetFrame(EulerZYX(Vec3(0, 0, 0.0)));
		mRobot->SetBaseLink(base);
		mRobot->SetBaseLinkType(srSystem::FIXED);
		mRobot->SetSelfCollision(false);

#ifdef srLOG
		printf("urdf 파일을 srLib에서 쓸 수 있도록 변환했습니다.\n");
#endif
	}
}

#endif
