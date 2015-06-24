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

// robot_urdf.cpp
// Author: Daniel Cookson
// Maintainer: Ian McMahon <imcmahon@rethinkrobotics.com>

#include "robot_state_publisher/robot_urdf.h"
#include <urdf_parser/urdf_parser.h>

namespace robot_urdf {

// ----------------------------------------------------------------
// Helper functions for editing XML element strings.

static std::size_t xmlContentStart(const std::string & doc, const std::string & tag)
{
  std::string start_tag = std::string("<") + tag;
  std::size_t pos = doc.find(start_tag);
  pos = doc.find('>', pos);
  if (pos != std::string::npos)  ++pos;
  return pos;
}

static std::size_t xmlContentEnd(const std::string & doc, const std::string & tag)
{
  std::string end_tag = std::string("</") + tag;
  return doc.rfind(end_tag);
}

static void xmlInsertContent(const std::string & content, const std::string & tag, std::string * doc)
{
  std::size_t pos = xmlContentEnd(*doc, tag);
  if (pos != std::string::npos)
  {
    doc->insert(pos, content);
  }
  else
  {
    ROS_WARN("Could not insert XML content; end tag '%s' not found.", tag.c_str());
  }
}

std::string xmlGetContent(const std::string & doc, const std::string & tag)
{
  if (doc.empty())  return std::string();
  std::size_t startPos = xmlContentStart(doc, tag);
  std::size_t endPos = xmlContentEnd(doc, tag);
  if ((startPos == std::string::npos) ||
      (endPos == std::string::npos))
  {
    return std::string();
  }
  return doc.substr(startPos, endPos - startPos);
}


// ----------------------------------------------------------------
// RobotURDF


RobotURDF::RobotURDF()
    : m_urdfPtrFg(new urdf::Model())
    , m_urdfPtrBg(new urdf::Model())
    , m_valid(false)
    , m_updateCount(0)
{
}

bool RobotURDF::init()
{
  return init(std::string("/robot_base_description"));
}

bool RobotURDF::init(const std::string & urdfParamName)
{
  std::string urdfString;
  ros::NodeHandle handle;
  m_valid = false;

  if (handle.getParam(urdfParamName, urdfString))
  {
    m_urdfBase = urdfString;
    m_valid =
        m_urdfPtrFg->initString(urdfString) &&
        m_urdfPtrBg->initString(urdfString);
    if (m_valid)
    {
      regenerateUrdf();
      swap();
      regenerateUrdf();
      swap();

      ROS_INFO("RobotURDF:  Subscribing to /robot/urdf");
      ros::NodeHandle handle("/robot");
      m_URDFConfigurationSubscriber =
          handle.subscribe("urdf", 10, &RobotURDF::onURDFConfigurationMsg, this,
                           ros::TransportHints().tcpNoDelay());
    }
    else
    {
      ROS_ERROR("RobotURDF:  Failed to parse urdf.");
    }
  }
  else
  {
    ROS_ERROR("RobotURDF: Parameter %s could not be found/read", urdfParamName.c_str());
  }

  return m_valid;
}

void RobotURDF::setRobotDescription()
{
  // Only one node should do this -- it takes 12 ms.
  ROS_INFO("Saving the URDF to the parameter server");
  ros::param::set("/robot_description", m_urdfDoc);
}

void RobotURDF::loadUrdfFragmentParam(const std::string & paramName,
                                      const std::string & linkName,
                                      const std::string & jointName)
{
  std::string urdfString;
  ros::NodeHandle handle;

  // It is expected that on a given type of robot this parameter may not be found,
  // for example the left gripper on a right-armed robot.
  if (handle.getParam(paramName, urdfString))
  {
    URDFFragment & fragment = m_urdfMap[makeKey(linkName, jointName)];
    fragment.parentLink = linkName;
    fragment.jointName = jointName;
    // Store just the content of the XML fragment -- expected to be found in a "robot" element:
    fragment.xml = xmlGetContent(urdfString, "robot");
    fragment.timestamp = ros::Time::now().toSec();
  }
}


bool RobotURDF::regenerateUrdf()
{
  static std::string root("robot");  // root element tag

  // Copy the base document:
  m_urdfDoc = m_urdfBase;
  //for (auto & pair : m_urdfMap)
  for (URDFFragmentMap::iterator pair = m_urdfMap.begin(); pair != m_urdfMap.end(); pair++)
  {
    // insert the children of each fragment into the document:
    std::string & frag = pair->second.xml;
    if (!frag.empty())
    {
      // Insert the fragment content into the base XML document:
      xmlInsertContent(frag, root, &m_urdfDoc);
    }
  }
  try
  {
    m_urdfPtrBg->initString(m_urdfDoc);
    // TODO: handle invalid doc
  }
  catch(std::exception & e)
  {
    ROS_ERROR("Could not regenerate URDF: %s", e.what());
    return false;
  }

  return true;
}


// URDFConfiguration subscriber callback.
void RobotURDF::onURDFConfigurationMsg(const robot_state_publisher::URDFConfiguration &config)
{
  const std::string & linkName = config.link;
  if (linkName.empty())
  {
    ROS_WARN("RobotURDF: URDFConfiguration has an empty link name!");
    return;
  }
  const std::string & jointName = config.joint;
  if (jointName.empty())
  {
    ROS_WARN("RobotURDF: URDFConfiguration has an empty joint name!");
    return;
  }

  double configTimestamp = config.time.toSec();

  ROS_DEBUG("RobotURDF: URDFConfiguration %s/%s %f",
           linkName.c_str(), jointName.c_str(), configTimestamp);
  boost::unique_lock<boost::mutex> updateLock(m_updateMutex, boost::try_to_lock);

  std::string key = makeKey(linkName, jointName);
  URDFFragmentMap::iterator pair = m_urdfMap.find(key);
  if (!updateLock.owns_lock())
  {
    // Only report this when we actually need to update the URDF.
    if ((pair != m_urdfMap.end()) &&
        (configTimestamp > pair->second.timestamp))
    {
      // It's OK -- unless something is seriously broken we'll get the lock the next time around (or the next).
      ROS_INFO("RobotURDF: URDFConfiguration update %s (%f) failed to acquire update lock.",
               key.c_str(), configTimestamp);
    }
    return;
  }
  double startChange = ros::Time::now().toSec();  // Time URDF updates.

  URDFFragment & fragment = m_urdfMap[key];
  if (configTimestamp > fragment.timestamp)
  {
    ROS_INFO("RobotURDF:  URDFConfiguration update #%d, %s (%f > %f)",
              m_updateCount, key.c_str(), configTimestamp, fragment.timestamp);
    std::string oldXml = fragment.xml;            // In case we have to revert it.
    double oldTimestamp = fragment.timestamp;     // In case we have to revert it.
    fragment.parentLink = linkName;
    fragment.jointName = jointName;
    // Store just the content of the XML fragment -- expected to be found in a "robot" element:
    //fragment.xml = xmlGetContent(hu::URDF::jsonToUrdf(config.urdf), "robot");
    fragment.xml = xmlGetContent(config.urdf, "robot");
    // Note that if the fragment is empty (i.e., deleted) we still want to keep
    //  it so we don't handle the message again.

    fragment.timestamp = configTimestamp;

    if (fragment.xml.empty() && !config.urdf.empty())
    {
      ROS_ERROR("URDFConfiguration failed; invalid JSON urdf:\n%s\n",
                config.urdf.c_str());
      m_valid = false;
      return;
    }

    // Update resources in the background in response to the URDF change:
    m_valid = onURDFChange(linkName);
    if (m_valid)
    {
      BOOST_SIGNAL_MEMBER(this, Changed)(linkName);

      // As quickly as possible, swap updated background resources into the foreground:
      boost::unique_lock<boost::shared_mutex> swapLock(m_swapMutex, boost::try_to_lock);
      if (swapLock.owns_lock())
      {
        // Swap background/foreground:
        onURDFSwap(linkName);
        BOOST_SIGNAL_MEMBER(this, Swapped)(linkName);
        ++m_updateCount;  // debug
      }
      else
      {
        ROS_INFO("RobotURDF: URDFConfiguration update %s (%f) failed to acquire swap lock.",
                 key.c_str(), configTimestamp);
        // It's OK -- unless something is seriously broken we'll get the lock the next time around (or the next).
        // It shouldn't matter that onURDFChanged is not undone -- that should only have affected background resources.
        m_valid = false;
      }
      ROS_INFO("URDFChange time: %f", ros::Time::now().toSec() - startChange);
    }
    else
    {
      ROS_ERROR("RobotURDF: URDFConfiguration update %s (%f) failed!",
                key.c_str(), configTimestamp);
    }


    if (!m_valid)
    {
      // When the update fails restore the cached old data.
      fragment.timestamp = oldTimestamp;
      fragment.xml = oldXml;
    }
  }
}

// Invoked when the URDF changes in response to a URDFConfiguration message.
// This should only affect background resources.
bool RobotURDF::onURDFChange(const std::string &link_name)
{
  // Overriding subclasses must call this.
  m_valid = regenerateUrdf();
  return m_valid;
}

// Invoked after the URDF changes to swap resources.  This is a performance enhancement.
void RobotURDF::onURDFSwap(const std::string &link_name)
{
  // Overriding subclasses must call this.
  swap();
}

}  // namespace robot_urdf
