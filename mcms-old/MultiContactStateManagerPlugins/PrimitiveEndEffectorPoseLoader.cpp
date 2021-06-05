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
#include <MotionGenerator/TrajectoryGenerator/OrientationTrajectoryGenerator.h>

#include <Math/Physics.h>
#include <Util/MiscString.h>
#include <Util/yaml/YamlReader.h>

#include "PrimitiveEndEffectorPoseLoader.h"
#include "PrimitiveEndEffectorPose.h"

using namespace hrp;
using namespace common_interface;
using namespace multi_contact_state_manager;
using namespace trajectory_generator;

PrimitiveEndEffectorPoseLoader::PrimitiveEndEffectorPoseLoader(TaskExecutionManager* parentManager,
							       const char* name,
							       std::list<std::string>* cmd_init) :
  MultiContactStatePrimitiveLoaderBase(parentManager, name, cmd_init)
{
  initialCommands(cmd_init);
}

MultiContactStatePrimitiveBase* PrimitiveEndEffectorPoseLoader::
loadPrimitive(const std::string& stateId, const std::string& guid,
              MultiContactStatePrimitiveGroup* mcsGroup, MultiContactStateTransition* parentState,
              const cnoid::YamlMapping& archive, std::ostringstream& o_strm)
{
  MultiContactStateManager* mcsManager
    = dynamic_cast<MultiContactStateManager*>(getParentTaskExecutionManager());
  if (!mcsManager)
    return NULL;

  PrimitiveEndEffectorPose* mcs = new PrimitiveEndEffectorPose(mcsManager, mcsGroup, this,
							       parentState, stateId, guid);

  // positionInterpolator

  const cnoid::YamlNodePtr& node_positionInterpolator = archive.find("positionInterpolator");
  PositionTrajectoryGenerator* positionInterpolator = NULL;
  if (node_positionInterpolator != NULL) {
    if (node_positionInterpolator->isValid()) {
      std::string name_positionInterpolator = node_positionInterpolator->toString();
      if (!name_positionInterpolator.empty()) {
        positionInterpolator = hmc->findTaskExecution<PositionTrajectoryGenerator*>(name_positionInterpolator);
        if (positionInterpolator) {
          mcs->registerPositionInterpolator(positionInterpolator);
          o_strm << "set positionInterpolator: " << positionInterpolator->getTaskName() << std::endl;
        }
        else
          o_strm << "\e[1;31m!!!ERROR!!!\e[m Can't find the positionInterpolator in "
                 << mcs->getName() << std::endl;
      }
      else
        o_strm << "\e[1;31m!!!ERROR!!!\e[m positionInterpolator is not specified defined in "
               << mcs->getName() << std::endl;
    }
    else
      o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid positionInterpolator data in "
             << mcs->getName() << std::endl;
  }
  else
    o_strm << "\e[1;31m!!!ERROR!!!\e[m positionInterpolator is not defined in "
           << mcs->getName() << std::endl;
  
  // orientationInterpolator

  const cnoid::YamlNodePtr& node_orientationInterpolator = archive.find("orientationInterpolator");
  OrientationTrajectoryGenerator* orientationInterpolator = NULL;
  if (node_orientationInterpolator != NULL) {
    if (node_orientationInterpolator->isValid()) {
      std::string name_orientationInterpolator = node_orientationInterpolator->toString();
      if (!name_orientationInterpolator.empty()) {
        orientationInterpolator = hmc->findTaskExecution<OrientationTrajectoryGenerator*>(name_orientationInterpolator);
        if (orientationInterpolator) {
          mcs->registerOrientationInterpolator(orientationInterpolator);
          o_strm << "set orientationInterpolator: " << orientationInterpolator->getTaskName() << std::endl;
        }
        else
          o_strm << "\e[1;31m!!!ERROR!!!\e[m Can't find the orientationInterpolator in "
		 << mcs->getName() << std::endl;
      }
      else
	o_strm << "\e[1;31m!!!ERROR!!!\e[m orientationInterpolator is not specified defined in "
	       << mcs->getName() << std::endl;
    }
    else
      o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid orientationInterpolator data in "
	     << mcs->getName() << std::endl;
  }
  else
    o_strm << "\e[1;31m!!!ERROR!!!\e[m orientationInterpolator is not defined in "
	   << mcs->getName() << std::endl;

  if (positionInterpolator && orientationInterpolator) {
    
    // poses
    
    const cnoid::YamlSequencePtr& seq_poses = archive.findSequence("poses");
    if (seq_poses != NULL) {
      for (int j = 0; j < seq_poses->size(); j++) {
        const cnoid::YamlMapping& archive_poses = *(*seq_poses)[j].toMapping();
        // pos
        const cnoid::YamlSequencePtr seq_pos = archive_poses.findSequence("pos");
        Vector3 pos  = Vector3::Zero();
        if (seq_pos != NULL) {
          if (seq_pos->isValid()) {
            pos  << (*seq_pos)[X].toDouble(), (*seq_pos)[Y].toDouble(), (*seq_pos)[Z].toDouble();
            mcs->addDesPosition(pos);
          }
          else
            o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid pos data in "
                   << mcs->getName() << std::endl;
        }
        else
          o_strm << "\e[1;31m!!!ERROR!!!\e[m pos is not defined in "
                 << mcs->getName() << std::endl;
        // rpy
        const cnoid::YamlSequencePtr seq_rpy = archive_poses.findSequence("rpy");
        Vector3 rpy  = Vector3::Zero();
        if (seq_rpy != NULL) {
          if (seq_rpy->isValid()) {
            rpy  << (*seq_rpy)[X].toDouble(), (*seq_rpy)[Y].toDouble(), (*seq_rpy)[Z].toDouble();
            mcs->addDesOrientation(DegToRad * rpy);
          }
          else
            o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid rpy data in "
                   << mcs->getName() << std::endl;
        }
        else
          o_strm << "\e[1;31m!!!ERROR!!!\e[m rpy is not defined in "
                 << mcs->getName() << std::endl;
      }
    }
    else
      o_strm << "\e[1;31m!!!ERROR!!!\e[m poses is not defined in "
	     << mcs->getName() << std::endl;

    // finalLinearVelocity

    const cnoid::YamlSequencePtr& seq_finalLinearVelocity = archive.findSequence("finalLinearVelocity");
    Vector3 vf  = Vector3::Zero();
    if (seq_finalLinearVelocity != NULL) {
      if (seq_finalLinearVelocity->isValid()) {
        vf <<
          (*seq_finalLinearVelocity)[X].toDouble(),
          (*seq_finalLinearVelocity)[Y].toDouble(),
          (*seq_finalLinearVelocity)[Z].toDouble();
        mcs->setDesFinalLinearVelocity(vf);
      }
      else
        o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid finalLinearVelocity data in "
               << mcs->getName() << std::endl;
    }

    // finalAngularVelocity

    const cnoid::YamlSequencePtr& seq_finalAngularVelocity = archive.findSequence("finalAngularVelocity");
    Vector3 wf  = Vector3::Zero();
    if (seq_finalAngularVelocity != NULL) {
      if (seq_finalAngularVelocity->isValid()) {
        wf <<
          (*seq_finalAngularVelocity)[X].toDouble(),
          (*seq_finalAngularVelocity)[Y].toDouble(),
          (*seq_finalAngularVelocity)[Z].toDouble();
        mcs->setDesFinalAngularVelocity(wf);
      }
      else
        o_strm << "\e[1;31m!!!ERROR!!!\e[m invalid finalAngularVelocity data in "
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
          mcs->setPositionInterpolationTiming(t0, tf);
          mcs->setOrientationInterpolationTiming(t0, tf);
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
