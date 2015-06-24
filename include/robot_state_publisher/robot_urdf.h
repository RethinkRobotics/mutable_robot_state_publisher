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

// robot_urdf.h
// Author: Daniel Cookson
// Maintainer: Ian McMahon <imcmahon@rethinkrobotics.com>

#ifndef ROBOT_URDF_H_
#define ROBOT_URDF_H_

#include <map>
#include <vector>
#include <stdint.h>
#include <string>
#include <boost/thread/shared_mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/signals2.hpp>
#include <ros/ros.h>

#include "urdf/model.h"

#include <kdl/kdl.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>

#include <robot_state_publisher/URDFConfiguration.h>

namespace robot_urdf {

/**
 * \def BOOST_SIGNAL_DEFINE(name, returnType, handlerPrototype)
 * Convenience macro to define a signal as a class member.
 *    
 * The macro does three things:
 * 1. Define a public typedef called [name]Signal which is of type boost::signals2::signal<returnType(handlerPrototype)> 
 * 2. Define a protected [name]Signal member variable called m_signal[name]
 * 3. Define a public inline function called get[name]Signal() which returns a reference to m_signal[name].   
 *  
 * For example:
 *  
 * \code
 * class MyClass {
 *   BOOST_SIGNAL_DEFINE(Feeling, void, (std::string));
 *   ...
 * }
 * \endcode
 *
 * defines a class which holds a signal named Feeling whose form is a void function accepting
 * a string.
 * \param name the name of the signal to define.  Use this with BOOST_SIGNAL_CONNECT() and BOOST_SIGNAL_MEMBER().
 * \param handlerPrototype describes how the signal handlers will be invoked.
 *
 */
#define BOOST_SIGNAL_DEFINE(name,returnType,handlerPrototype) \
  public: \
    typedef boost::signals2::signal<returnType handlerPrototype> name ## Signal; \
  protected: \
    name ## Signal m_signal ## name; \
  public: \
    inline name ## Signal & get ## name ## Signal() { return m_signal ## name ; }
/**
 * \ingroup libutilities_signals
 * \def BOOST_SIGNAL_MEMBER(obj,name)
 * gets the signal of specified name which is a member of the specified object
 *
 * Typical usage is in your header file you have defined your signal with the
 * BOOST_SIGNAL_DEFINE macro and now wish to emit the signal from within the class.
 *
 * Say you have defined
 * \code
 * BOOST_SIGNAL_DEFINE(Feeling, void, (std::string))
 * \endcode
 * in your header.
 *
 * Now within your class you emit that signal via
 *
 * \code
 * BOOST_SIGNAL_MEMBER(this,Feeling)("great")
 * \endcode
 */
#define BOOST_SIGNAL_MEMBER(objectWithSignal, name) \
  (objectWithSignal)->get ## name ## Signal()
//TODO: Remove Macro


/** Container for the URDF model associated with a robot.
 *  Subscribes to URDFConfiguration messages and updates the model accordingly.
 */
class RobotURDF
{
 public:
  /**
   * Emitted when the URDF changes.
   */
  BOOST_SIGNAL_DEFINE(Changed, void, (const std::string& link_name));

  /**
   * Emitted after the URDF changes to swap resources.
   */
  BOOST_SIGNAL_DEFINE(Swapped, void, (const std::string& link_name));

 public:
  typedef boost::shared_ptr<urdf::Model> UrdfPtr;
  typedef boost::shared_ptr<const urdf::Model> ConstUrdfPtr;

  RobotURDF();

  bool init();  // Load the default urdf parameter
  bool init(const std::string & urdfParamName);


  bool isValid() const { return m_valid; }


  ConstUrdfPtr getUrdfPtr() { return m_urdfPtrFg; }
  ConstUrdfPtr getUrdfBgPtr() { return m_urdfPtrBg; }

  // Write the current URDF to the /robot_description parameter:
  void setRobotDescription();

 protected:
  typedef struct
  {
    double      timestamp;     // Note that 0.0 is an invalid timestamp
    std::string parentLink;
    std::string jointName;
    std::string xml;
  } URDFFragment;
  typedef std::map<std::string, URDFFragment> URDFFragmentMap;

  URDFFragmentMap m_urdfMap;
  std::string     m_urdfBase;    // Base URDF document
  std::string     m_urdfDoc;     // Current URDF document, for reference

  UrdfPtr m_urdfPtrFg;
  UrdfPtr m_urdfPtrBg;
  void swap()  {  m_urdfPtrFg.swap(m_urdfPtrBg); }

  bool m_valid;
  uint32_t m_updateCount;  // debug
  ros::Subscriber         m_URDFConfigurationSubscriber;

  static std::string makeKey(const std::string & linkName, const std::string & jointName)
    {  return linkName + "/" + jointName;  }

  void loadUrdfFragmentParam(const std::string & paramName,
                             const std::string & linkName,
                             const std::string & jointName);

  bool regenerateUrdf();

  void onURDFConfigurationMsg(const robot_state_publisher::URDFConfiguration &config);
  virtual bool onURDFChange(const std::string &link_name);
  virtual void onURDFSwap(const std::string &link_name);

 public:
  mutable boost::shared_mutex m_swapMutex;    // Protect access while swapping shared pointers.
  mutable boost::mutex m_updateMutex;         // Protect access while updating the URDF and things that depend on it.
};


}  // namespace robot_urdf

#endif /* ROBOT_URDF_H_ */
