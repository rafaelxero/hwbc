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
#include <MotionGenerator/TrajectoryGenerator/PositionTrajectoryGenerator.h>

#include <Math/Physics.h>
#include <Util/MiscString.h>
#include <Util/yaml/YamlReader.h>

#include "PrimitiveComxyPzLoader.h"
#include "PrimitiveComxyPz.h"

using namespace hrp;
using namespace common_interface;
using namespace multi_contact_state_manager;
using namespace trajectory_generator;

PrimitiveComxyPzLoader::PrimitiveComxyPzLoader(TaskExecutionManager* parentManager,
					       const char* name,
					       std::list<std::string>* cmd_init) :
  MultiContactStatePrimitiveLoaderBase(parentManager, name, cmd_init)
{
  initialCommands(cmd_init);
}

MultiContactStatePrimitiveBase* PrimitiveComxyPzLoader::
loadPrimitive(const std::string& stateId, const std::string& guid,
              MultiContactStatePrimitiveGroup* mcsGroup, MultiContactStateTransition* parentState,
              const cnoid::YamlMapping& archive, std::ostringstream& o_strm)
{
  MultiContactStateManager* mcsManager
    = dynamic_cast<MultiContactStateManager*>(getParentTaskExecutionManager());
  if (!mcsManager)
    return NULL;

  PrimitiveComxyPz* mcs = new PrimitiveComxyPz(mcsManager, mcsGroup, this,
                                               parentState, stateId, guid);

  // comInterpolator
  
  const cnoid::YamlNodePtr& node_comInterpolator = archive.find("comInterpolator");
  PositionTrajectoryGenerator* comInterpolator = NULL;
  if (node_comInterpolator != NULL) {
    if (node_comInterpolator->isValid()) {
      std::string name_comInterpolator = node_comInterpolator->toString();
      if (!name_comInterpolator.empty()) {
        comInterpolator = hmc->findTaskExecution<PositionTrajectoryGenerator*>(name_comInterpolator);
        if (comInterpolator) {
          mcs->registerComInterpolator(comInterpolator);
          o_strm << "set comInterpolator: " << comInterpolator->getTaskName() << std::endl;
        }
        else
          o_strm << "\e[1;31m!!!ERROR!!!\e[m Can't find the comInterpolator in "
                 << mcs->getName() << std::endl;
      }
      else
	o_strm << "\e[1;31m!!!ERROR!!!\e[m comInterpolator is not specified defined in "
         << mcs->getName() << std::endl;
    }
    else
      o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid comInterpolator data in "
             << mcs->getName() << std::endl;
  }
  else
    o_strm << "\e[1;31m!!!ERROR!!!\e[m comInterpolator is not defined in "
           << mcs->getName() << std::endl;
  
  // bodyPositionInterpolator

  const cnoid::YamlNodePtr& node_bodyPositionInterpolator = archive.find("bodyPositionInterpolator");
  PositionTrajectoryGenerator* bodyPositionInterpolator = NULL;
  if (node_bodyPositionInterpolator != NULL) {
    if (node_bodyPositionInterpolator->isValid()) {
      std::string name_bodyPositionInterpolator = node_bodyPositionInterpolator->toString();
      if (!name_bodyPositionInterpolator.empty()) {
	bodyPositionInterpolator = hmc->findTaskExecution<PositionTrajectoryGenerator*>(name_bodyPositionInterpolator);
	if (bodyPositionInterpolator) {
	  mcs->registerBodyPositionInterpolator(bodyPositionInterpolator);
	  o_strm << "set bodyPositionInterpolator: " << bodyPositionInterpolator->getTaskName() << std::endl;
	}
	else
	  o_strm << "\e[1;31m!!!ERROR!!!\e[m Can't find the bodyPositionInterpolator in "
           << mcs->getName() << std::endl;
      }
      else
	o_strm << "\e[1;31m!!!ERROR!!!\e[m bodyPositionInterpolator is not specified defined in "
         << mcs->getName() << std::endl;
    }
    else
      o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid bodyPositionInterpolator data in "
             << mcs->getName() << std::endl;
  }
  else
    o_strm << "\e[1;31m!!!ERROR!!!\e[m bodyPositionInterpolator is not defined in "
           << mcs->getName() << std::endl;

  // zmpInterpolator
  
  const cnoid::YamlNodePtr& node_zmpInterpolator = archive.find("zmpInterpolator");
  PositionTrajectoryGenerator* zmpInterpolator = NULL;
  if (node_zmpInterpolator != NULL) {
    if (node_zmpInterpolator->isValid()) {
      std::string name_zmpInterpolator = node_zmpInterpolator->toString();
      if (!name_zmpInterpolator.empty()) {
        zmpInterpolator = hmc->findTaskExecution<PositionTrajectoryGenerator*>(name_zmpInterpolator);
        if (zmpInterpolator) {
          mcs->registerZmpInterpolator(zmpInterpolator);
          o_strm << "set zmpInterpolator: " << zmpInterpolator->getTaskName() << std::endl;
        }
        else
          o_strm << "\e[1;31m!!!ERROR!!!\e[m Can't find the zmpInterpolator in "
                 << mcs->getName() << std::endl;
      }
      else
	o_strm << "\e[1;31m!!!ERROR!!!\e[m zmpInterpolator is not specified defined in "
         << mcs->getName() << std::endl;
    }
    else
      o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid zmpInterpolator data in "
             << mcs->getName() << std::endl;
  }
  else
    o_strm << "\e[1;31m!!!ERROR!!!\e[m zmpInterpolator is not defined in "
           << mcs->getName() << std::endl;
  

  if (comInterpolator && bodyPositionInterpolator && zmpInterpolator) {
  
    // positions
  
    const cnoid::YamlSequencePtr& seq_positions = archive.findSequence("positions");
    if (seq_positions != NULL) {
      for (int j = 0; j < seq_positions->size(); j++) {
        const cnoid::YamlMapping& archive_positions = *(*seq_positions)[j].toMapping();
        // pos
        const cnoid::YamlSequencePtr seq_pos = archive_positions.findSequence("pos");
        Vector3 com_pos  = Vector3::Zero();
        Vector3 body_pos = Vector3::Zero();
        Vector3 zmp_pos  = Vector3::Zero();
        if (seq_pos != NULL) {
          if (seq_pos->isValid()) {
            com_pos  << (*seq_pos)[X].toDouble(), (*seq_pos)[Y].toDouble(), 0.0;
            body_pos << 0.0, 0.0, (*seq_pos)[Z].toDouble();
            zmp_pos  << (*seq_pos)[X].toDouble(), (*seq_pos)[Y].toDouble(), 0.0;
            mcs->addComDesPosition(com_pos);
            mcs->addBodyDesPosition(body_pos);
            mcs->addZmpDesPosition(zmp_pos);
          }
          else
            o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid pos data in "
                   << mcs->getName() << std::endl;
        }
        else
          o_strm << "\e[1;31m!!!ERROR!!!\e[m pos is not defined in "
                 << mcs->getName() << std::endl;
      }
    }
    else
      o_strm << "\e[1;31m!!!ERROR!!!\e[m positions is not defined in "
             << mcs->getName() << std::endl;

    // finalLinearVelocity

    const cnoid::YamlSequencePtr& seq_finalLinearVelocity = archive.findSequence("finalLinearVelocity");
    Vector3 com_vf  = Vector3::Zero();
    Vector3 body_vf = Vector3::Zero();
    Vector3 zmp_vf  = Vector3::Zero();
    if (seq_finalLinearVelocity != NULL) {
      if (seq_finalLinearVelocity->isValid()) {
        com_vf << (*seq_finalLinearVelocity)[X].toDouble(), (*seq_finalLinearVelocity)[Y].toDouble(), 0.0;
        body_vf << 0.0, 0.0, (*seq_finalLinearVelocity)[Z].toDouble();
        zmp_vf << (*seq_finalLinearVelocity)[X].toDouble(), (*seq_finalLinearVelocity)[Y].toDouble(), 0.0;
        mcs->setComDesFinalLinearVelocity(com_vf);
        mcs->setBodyDesFinalLinearVelocity(body_vf);
        mcs->setZmpDesFinalLinearVelocity(zmp_vf);
      }
      else
        o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid finalLinearVelocity data in "
               << mcs->getName() << std::endl;
    }

    // duration

    const cnoid::YamlNodePtr& node_duration = archive.find("duration");
    if (node_duration != NULL) {
      const cnoid::YamlMapping& archive_duration = *(*node_duration).toMapping();
      // t0
      const cnoid::YamlNodePtr node_t0 = archive_duration.find("t0");
      double t0 = 0.0;
      if (node_t0 != NULL) {
        if (node_t0->isValid()) {
          t0 = node_t0->toDouble();
          if (t0 < 0.0) {
            t0 = 0.0;
            o_strm << "!!!WARNING!!! t0 should be greater or equal than 0 in "
                   << mcs->getName() << ", setting it as 0.0" << std::endl;
          }
        }
        else
          o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid t0 data in "
                 << mcs->getName() << std::endl;
      }
      // tf
      const cnoid::YamlNodePtr node_tf = archive_duration.find("tf");
      double tf = 0.0;
      if (node_tf != NULL) {
        if (node_tf->isValid()) {
          tf = node_tf->toDouble();
          if (tf < t0) {
            tf = t0;
            o_strm << "!!!WARNING!!! tf should be greater or equal than t0 in "
                   << mcs->getName() << ", setting it equal to t0" << std::endl;
          }
          mcs->setComInterpolationTiming(t0, tf);
          mcs->setBodyPositionInterpolationTiming(t0, tf);
          mcs->setZmpInterpolationTiming(t0, tf);
        }
        else
          o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid tf data in "
                 << mcs->getName() << std::endl;
      }
    }
  }
  
  MultiContactStatePrimitiveLoaderBase::loadPrimitive(mcs, mcs->getName(), archive, o_strm);
  
  return mcs;
}
