#ifndef DRACO_H
#define DRACO_H

#include <map>
#include "srDyn/srSpace.h"
#include <vector>

class srDraco: public srSystem{
 public:
  srDraco(Vec3 location);
  virtual ~srDraco();

  std::vector<srCollision*> collision_;
 private:

  void _SetBase(Vec3 location, srSystem::BASELINKTYPE base_link_type);
  int num_r_joint_;
  int num_p_joint_;
  int num_fixed_joint_;
  int RJidx_;
  int WJidx_;

  std::vector<srLink*> link_;
  std::vector<srRevoluteJoint*> r_joint_;
  std::vector<srPrismaticJoint*> p_joint_;
  std::vector<srWeldJoint*> fixed_joint_;
  std::vector<srPrismaticJoint*> vp_joint_;
  std::vector<srRevoluteJoint*> vr_joint_;
  std::vector<srLink*> v_link_;
  std::map<std::string, int> fixed_joint_idx_map_;
  std::map<std::string, int> p_joint_idx_map_;
  std::map<std::string, int> r_joint_idx_map_;
  std::map<std::string, int> link_idx_map_;

	int num_act_joint_;


  void _SetCollision();
  void _SetInitialConf();
  void _SetJointLimit();
  void _SetInertia();

  void _AssembleRobot(Vec3 location);
  void _DefineLinks();
  void _DefineVirtualJoint();
  void _DefineActuatedJoint();
};

#endif
