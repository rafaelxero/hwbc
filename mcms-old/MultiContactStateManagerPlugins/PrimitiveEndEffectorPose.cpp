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

#include <MotionGenerator/MultiContactStateManager/MultiContactStateManager.h>
#include <MotionGenerator/TrajectoryGenerator/PositionTrajectoryGenerator.h>
#include <MotionGenerator/TrajectoryGenerator/OrientationTrajectoryGenerator.h>
#include "PrimitiveEndEffectorPose.h"

using namespace hrp;
using namespace multi_contact_state_manager;
using namespace trajectory_generator;

PrimitiveEndEffectorPose::PrimitiveEndEffectorPose(MultiContactStateManager* mcsManager,
						   MultiContactStatePrimitiveGroup* mcsGroup,
						   MultiContactStatePrimitiveLoaderBase* loader,
						   MultiContactStateTransition* parentState,
						   const std::string& name, const std::string& guid) :

  MultiContactStatePrimitiveBase(mcsManager, mcsGroup, loader, parentState, name, guid),

  m_posInterpolationTiming({0.0, 0.0}),
  m_desFinalLinearVelocity(Vector3::Zero()),
  
  m_rpyInterpolationTiming({0.0, 0.0}),
  m_desFinalAngularVelocity(Vector3::Zero())
{}

void PrimitiveEndEffectorPose::onEntry(void)
{
  if (m_position_interpolator && m_desPositions.size() > 0) {
    
    m_position_interpolator->configure_timing(m_posInterpolationTiming.t0,
					      m_posInterpolationTiming.tf);
    m_position_interpolator->set_positions(m_desPositions);
    m_position_interpolator->set_final_linear_velocity(m_desFinalLinearVelocity);
  }
  
  if (m_orientation_interpolator && m_desOrientations.size() > 0) {
    m_orientation_interpolator->configure_timing(m_rpyInterpolationTiming.t0,
						 m_rpyInterpolationTiming.tf);
    m_orientation_interpolator->set_orientations(m_desOrientations);
    m_orientation_interpolator->set_final_angular_velocity(m_desFinalAngularVelocity);
  }
}

void PrimitiveEndEffectorPose::onExit(void)
{
  m_posInterpolationTiming = {0.0, 0.0};
  m_desPositions.clear();
  m_desFinalLinearVelocity.setZero();

  m_rpyInterpolationTiming = {0.0, 0.0};
  m_desOrientations.clear();
  m_desFinalAngularVelocity.setZero();
}

bool PrimitiveEndEffectorPose::check(int n_time,
                                     const std::bitset<NUM_MULTI_CONTACT_STATE_STOP_ACTIONS>& signal_state)
{
  return (m_position_interpolator->is_done() &&
          m_orientation_interpolator->is_done());
}
