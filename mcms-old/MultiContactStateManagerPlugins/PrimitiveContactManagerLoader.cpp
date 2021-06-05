// -*- mode: c++; indent-tabs-mode: nil; tab-width: 2; c-basic-offset: 2; -*-
/*
 * Copyright (c) 2019,
 * @author Rafael Cisneros
 *
 * AIST
 *
 * All rights reserved.
 *
 * This program is made available under the terms of the Eclipse Public License
 * v1.0 which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 */

#include <HumanoidMotionControlCore/HumanoidMotionControlCore.h>
#include <MotionGenerator/MultiContactStateManager/MultiContactStateManager.h>
#include <MultiContactMotionSolver/Constraints/ContactConstraintManager.h>

#include <Math/Physics.h>
#include <Util/MiscString.h>
#include <Util/yaml/YamlReader.h>

#include "PrimitiveContactManagerLoader.h"
#include "PrimitiveContactManager.h"

using namespace hrp;
using namespace common_interface;
using namespace multi_contact_state_manager;
using namespace multi_contact_motion_solver;

PrimitiveContactManagerLoader::PrimitiveContactManagerLoader(TaskExecutionManager* parentManager,
							       const char* name,
							       std::list<std::string>* cmd_init) :
  MultiContactStatePrimitiveLoaderBase(parentManager, name, cmd_init)
{
  initialCommands(cmd_init);
}

MultiContactStatePrimitiveBase* PrimitiveContactManagerLoader::
loadPrimitive(const std::string& stateId, const std::string& guid,
              MultiContactStatePrimitiveGroup* mcsGroup, MultiContactStateTransition* parentState,
              const cnoid::YamlMapping& archive, std::ostringstream& o_strm)
{
  MultiContactStateManager* mcsManager
    = dynamic_cast<MultiContactStateManager*>(getParentTaskExecutionManager());
  if (!mcsManager)
    return NULL;

  PrimitiveContactManager* mcs = new PrimitiveContactManager(mcsManager, mcsGroup, this,
							     parentState, stateId, guid);
  
  // contactManager

  const cnoid::YamlNodePtr& node_contactManager = archive.find("contactManager");
  ContactConstraintManager* contactManager = NULL;
  if (node_contactManager != NULL) {
    if (node_contactManager->isValid()) {
      std::string name_contactManager = node_contactManager->toString();
      if (!name_contactManager.empty()) {
        contactManager = hmc->findTaskExecution<ContactConstraintManager*>(name_contactManager);
        if (contactManager) {
          mcs->registerContactManager(contactManager);
          o_strm << "set contactManager: " << contactManager->getTaskName() << std::endl;
        }
        else
          o_strm << "\e[1;31m!!!ERROR!!!\e[m Can't find the contactManager in "
                 << mcs->getName() << std::endl;
      }
      else
        o_strm << "\e[1;31m!!!ERROR!!!\e[m contactManager is not specified defined in "
               << mcs->getName() << std::endl;
    }
    else
      o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid contactManager data in "
             << mcs->getName() << std::endl;
  }
  else
    o_strm << "\e[1;31m!!!ERROR!!!\e[m contactManager is not defined in "
           << mcs->getName() << std::endl;

  if (contactManager) {

    // robotSurface

    const cnoid::YamlNodePtr& node_robotSurface = archive.find("robotSurface");
    if (node_robotSurface != NULL) {
      if (node_robotSurface->isValid()) {
	std::string robotSurface = node_robotSurface->toString();
	mcs->setRobotSurface(robotSurface);
      }
      else
	o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid robotSurface data in "
               << mcs->getName() << std::endl;
    }
    else
      o_strm << "\e[1;31m!!!ERROR!!!\e[m robotSurface is not defined in "
	     << mcs->getName() << std::endl;
    
    // envSurface

    const cnoid::YamlNodePtr& node_envSurface = archive.find("envSurface");
    if (node_envSurface != NULL) {
      if (node_envSurface->isValid()) {
	std::string envSurface = node_envSurface->toString();
	mcs->setEnvSurface(envSurface);
      }
      else
	o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid envSurface data in "
               << mcs->getName() << std::endl;
    }
    else
      o_strm << "\e[1;31m!!!ERROR!!!\e[m envSurface is not defined in "
	     << mcs->getName() << std::endl;

    // constrained

    const cnoid::YamlNodePtr& node_constrained = archive.find("constrained");
    if (node_constrained != NULL) {
      if (node_constrained->isValid()) {
	bool constrained = isOn(node_constrained->toString());
	mcs->setConstrained(constrained);
      }
      else
	o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid constrained data in "
               << mcs->getName() << std::endl;
    }
    else
      o_strm << "\e[1;31m!!!ERROR!!!\e[m constrained is not defined in "
	     << mcs->getName() << std::endl;

    // stiffness

    const cnoid::YamlNodePtr& node_stiffness = archive.find("stiffness");
    if (node_stiffness != NULL) {
      const cnoid::YamlMapping& archive_stiffness = *(*node_stiffness).toMapping();
      // gain
      const cnoid::YamlNodePtr node_gain = archive_stiffness.find("gain");
      if (node_gain != NULL) {
	if (node_gain->isValid()) {
	  double gain = node_gain->toDouble();
	  mcs->setStiffnessGain(gain);
	}
	else
	  o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid gain data in "
                 << mcs->getName() << std::endl;
      }
      // mask
      const cnoid::YamlSequencePtr seq_mask = archive_stiffness.findSequence("mask");
      if (seq_mask != NULL) {
	if (seq_mask->isValid()) {
	  dvector6 mask = dvector6::Zero();
	  mask <<
	    (*seq_mask)[0].toDouble(), (*seq_mask)[1].toDouble(), (*seq_mask)[2].toDouble(),
	    (*seq_mask)[3].toDouble(), (*seq_mask)[4].toDouble(), (*seq_mask)[5].toDouble(),
	  
	  mcs->setStiffnessMask(mask);
	}
	else
	  o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid mask data in "
                 << mcs->getName() << std::endl;
      }
    }
    
    // damping

    const cnoid::YamlNodePtr& node_damping = archive.find("damping");
    if (node_damping != NULL) {
      const cnoid::YamlMapping& archive_damping = *(*node_damping).toMapping();
      // gain
      const cnoid::YamlNodePtr node_gain = archive_damping.find("gain");
      if (node_gain != NULL) {
	if (node_gain->isValid()) {
	  double gain = node_gain->toDouble();
	  mcs->setDampingGain(gain);
	}
	else
	  o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid gain data in "
                 << mcs->getName() << std::endl;
      }
      // mask
      const cnoid::YamlSequencePtr seq_mask = archive_damping.findSequence("mask");
      if (seq_mask != NULL) {
	if (seq_mask->isValid()) {
	  dvector6 mask = dvector6::Zero();
	  mask <<
	    (*seq_mask)[0].toDouble(), (*seq_mask)[1].toDouble(), (*seq_mask)[2].toDouble(),
	    (*seq_mask)[3].toDouble(), (*seq_mask)[4].toDouble(), (*seq_mask)[5].toDouble(),
	  
	  mcs->setDampingMask(mask);
	}
	else
	  o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid mask data in "
                 << mcs->getName() << std::endl;
      }
    }
  }

  MultiContactStatePrimitiveLoaderBase::loadPrimitive(mcs, mcs->getName(), archive, o_strm);

  return mcs;
}
