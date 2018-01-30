#include "srRobot.h"

using namespace srrobot;

srRobot::srRobot()
{

}

srRobot::~srRobot()
{
	delete &mLink;
	delete &mCollision;
	delete &mJoint;
}

srLink * srrobot::srRobot::setLink(string linkname)
{
	map < string, srLink* >::iterator item;

	item = mLink.find(linkname);
	if (item != mLink.end())
	{
		cout << "link (" << linkname << ") already exists" << endl;
		
		return NULL;
	}
	else
	{
		srLink* link;
		link = new srLink();
		mLink.insert(make_pair(linkname, link));

		return link;
	}
}

srCollision * srrobot::srRobot::setCollision(string collisionname)
{
	map < string, srCollision* >::iterator item;

	item = mCollision.find(collisionname);
	if (item != mCollision.end())
	{
		cout << "collision (" << collisionname << ") already exists" << endl;

		return NULL;
	}
	else
	{
		srCollision* collision;
		collision = new srCollision();
		mCollision.insert(make_pair(collisionname, collision));

		return collision;
	}
}

srJoint * srrobot::srRobot::setJoint(string jointname, srJoint::JOINTTYPE jointType)
{
	map < string, srJoint* >::iterator item;

	item = mJoint.find(jointname);
	if (item != mJoint.end())
	{
		cout << "joint (" << jointname << ") already exists" << endl;

		return NULL;
	}
	else
	{
		srJoint* joint;
		switch (jointType)
		{
		case srJoint::JOINTTYPE::REVOLUTE:
			joint = new srRevoluteJoint();
			break;
		case srJoint::JOINTTYPE::BALL:
			joint = new srBallJoint();
			break;
		case srJoint::JOINTTYPE::PRISMATIC:
			joint = new srPrismaticJoint();
			break;
		case srJoint::JOINTTYPE::UNIVERSAL:
			joint = new srUniversalJoint();
			break;
		case srJoint::JOINTTYPE::WELD:
			joint = new srWeldJoint();
			break;
		default:
			return NULL;
		}
		mJoint.insert(make_pair(jointname, joint));

		return joint;
	}
}

srLink* srRobot::getLink(string linkname)
{
	map < string, srLink* >::iterator item;

	item = mLink.find(linkname);
	if (item == mLink.end())
	{
		cout << "link (" << linkname << ") doesn't exist" << endl;
		return NULL;
	}
	else
	{
		srLink* link;
		link = item->second;
		return link;
	}


	//srLink* link;
	//map < string, srLink* >::iterator item;

	//item = mLink.find(linkname);
	//if (item == mLink.end())
	//{
	//	link = new srLink();
	//	mLink.insert(make_pair(linkname, link));
	//}
	//else
	//{
	//	link = item->second;
	//}

	//return link;
}

srCollision* srRobot::getCollision(string collisionname)
{
	map < string, srCollision* >::iterator item;

	item = mCollision.find(collisionname);
	if (item == mCollision.end())
	{
		cout << "collision (" << collisionname << ") doesn't exist" << endl;
		return NULL;
	}
	else
	{
		srCollision* collision;
		collision = item->second;
		return collision;
	}



	//srCollision* collision;
	//map < string, srCollision* >::iterator item;

	//item = mCollision.find(collisionname);
	//if (item == mCollision.end())
	//{
	//	collision = new srCollision();
	//	mCollision.insert(make_pair(collisionname, collision));
	//}
	//else
	//{
	//	collision = item->second;
	//}

	//return collision;
}

srJoint* srRobot::getJoint(string jointname)
{
	map < string, srJoint* >::iterator item;

	item = mJoint.find(jointname);
	if (item == mJoint.end())
	{
		cout << "joint (" << jointname << ") doesn't exist" << endl;
		return NULL;
	}
	else
	{
		srJoint* joint;
		joint = item->second;
		return joint;
	}



	//srJoint* joint;
	//map < string, srJoint* >::iterator item;

	//item = mJoint.find(jointname);
	//if (item == mJoint.end())
	//{
	//	switch (jointType)
	//	{
	//	case srJoint::JOINTTYPE::REVOLUTE:
	//		joint = new srRevoluteJoint();
	//		break;
	//	case srJoint::JOINTTYPE::BALL:
	//		joint = new srBallJoint();
	//		break;
	//	case srJoint::JOINTTYPE::PRISMATIC:
	//		joint = new srPrismaticJoint();
	//		break;
	//	case srJoint::JOINTTYPE::UNIVERSAL:
	//		joint = new srUniversalJoint();
	//		break;
	//	case srJoint::JOINTTYPE::WELD:
	//		joint = new srWeldJoint();
	//		break;
	//	default:
	//		return NULL;
	//	}
	//	mJoint.insert(make_pair(jointname, joint));
	//}
	//else
	//{
	//	joint = item->second;
	//}

	//return joint;
}