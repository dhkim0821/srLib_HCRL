/*******************************************************************/
/*                                                                 */
/*    MADE BY: Keunjun Choi                                        */
/*    Date: 2014-12-14                                             */
/*                                NAMESPACE:     srrobot           */
/*                                                                 */
/*******************************************************************/

#ifndef __SR_ROBOT__
#define __SR_ROBOT__

#include <map>
#include <string>
#include <srDyn/srSpace.h>

using namespace std;

namespace srrobot
{
	class srRobot : public srSystem
	{
	public:
		map < string, srLink* > mLink;
		map < string, srCollision* > mCollision;
		map < string, srJoint* > mJoint;

	public:
		srRobot();
		~srRobot();

		srLink*			setLink(string);
		srCollision*	setCollision(string);
		srJoint*		setJoint(string, srJoint::JOINTTYPE jointType = srJoint::JOINTTYPE::WELD);

		// starts with 'get' below 3 functions..... gu rim! (set link/col/joint inside, used in srURDF)
		srLink*			getLink(string);
		srCollision*	getCollision(string);
		srJoint*		getJoint(string);
	};


}

#endif
