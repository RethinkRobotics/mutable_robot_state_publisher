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

// robot_kdl_tree.cpp
// Author: Daniel Cookson
// Maintainer: Ian McMahon <imcmahon@rethinkrobotics.com>

#include "robot_state_publisher/robot_kdl_tree.h"
#include <kdl_parser/kdl_parser.hpp>
#include <string>

namespace robot_kdl_tree {

// ----------------------------------------------------------------
// RobotKDLTree


RobotKDLTree::RobotKDLTree()
    : m_treeFg(new KDL::Tree())
    , m_treeBg(new KDL::Tree())
{
}


bool RobotKDLTree::init()
{
  if (RobotURDF::init())
  {
    return initFromURDF();
  }
  return false;
}

bool RobotKDLTree::init(const std::string & urdfParamName)
{
  if (RobotURDF::init(urdfParamName))
  {
    return initFromURDF();
  }
  return false;
}

bool RobotKDLTree::initFromURDF()
{
  if (getTreeFromURDF())
  {
    swap();
    getTreeFromURDF();
    return true;
  }
  return false;
}

bool RobotKDLTree::getTreeFromURDF()
{
  RobotURDF::ConstUrdfPtr urdfPtr = getUrdfBgPtr();
  if (urdfPtr == NULL)
  {
    ROS_ERROR("RobotKDLTree: NULL background URDF Ptr!");
    return false;
  }

  if (!m_valid)
  {
    ROS_ERROR("RobotKDLTree: invalid!");
    return false;
  }

  m_valid = kdl_parser::treeFromUrdfModel(*urdfPtr, *(m_treeBg.get()));
  if (!m_valid)
  {
    ROS_ERROR("RobotKDLTree: Failed to create KDL tree from URDF model");
  }

  return m_valid;
}


bool RobotKDLTree::onURDFChange(const std::string &link_name)
{
  if (!RobotURDF::onURDFChange(link_name))  return false;
  return getTreeFromURDF();
}

void RobotKDLTree::onURDFSwap(const std::string &link_name)
{
  RobotURDF::onURDFSwap(link_name);
  swap();
}

}  // namespace robot_kdl_tree
