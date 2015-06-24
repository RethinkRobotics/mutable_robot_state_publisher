/*********************************************************************
 # Copyright (c) 2008-2015, Rethink Robotics
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 #
 # 1. Redistributions of source code must retain the above copyright notice,
 #    this list of conditions and the following disclaimer.
 # 2. Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 # 3. Neither the name of the Rethink Robotics nor the names of its
 #    contributors may be used to endorse or promote products derived from
 #    this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 # AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 # ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 # LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 # CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 # SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 # INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 # CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 # ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 # POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// robot_kdl_tree.h
// Author: Daniel Cookson
// Maintainer: Ian McMahon <imcmahon@rethinkrobotics.com>

#ifndef ROBOT_KDL_TREE_H_
#define ROBOT_KDL_TREE_H_

#include <robot_state_publisher/robot_urdf.h>

namespace robot_kdl_tree {


/** Container for the URDF model and KDL Tree associated with a robot.
 * Updates the KDL Tree when the underlying URDF changes.
 */
class RobotKDLTree : public robot_urdf::RobotURDF
{
 public:
  typedef boost::shared_ptr<RobotKDLTree> Ptr;
  typedef boost::shared_ptr<KDL::Tree> KDLTreePtr;

  static Ptr create()
  {
    Ptr pRobotKDLTree= Ptr(new RobotKDLTree());
    if (pRobotKDLTree->init())
    {
      return pRobotKDLTree;
    }
    return Ptr();
  }

  static Ptr create(const std::string & paramName)
  {
    Ptr pRobotKDLTree= Ptr(new RobotKDLTree());
    if (pRobotKDLTree->init(paramName))
    {
      return pRobotKDLTree;
    }
    return Ptr();
  }

 protected:
  bool initFromURDF();
  bool getTreeFromURDF();

  RobotKDLTree();                             // The default constructer is for testing only

  void swap()  {  m_treeFg.swap(m_treeBg); }

 public:
  virtual bool init();
  virtual bool init(const std::string & urdfParamName);

  virtual bool onURDFChange(const std::string &link_name);
  virtual void onURDFSwap(const std::string &link_name);

  const KDL::Tree & getTree()  { return *(m_treeFg.get()); }
  const KDL::Tree & getBgTree()  { return *(m_treeBg.get()); }

 protected:
  KDLTreePtr  m_treeFg;
  KDLTreePtr  m_treeBg;
};


} // namespace robot_kdl_tree


#endif /* ROBOT_KDL_TREE_H_ */
