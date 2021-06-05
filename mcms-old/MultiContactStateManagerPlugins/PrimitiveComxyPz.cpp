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
#include "PrimitiveComxyPz.h"

using namespace hrp;
using namespace multi_contact_state_manager;
using namespace trajectory_generator;

PrimitiveComxyPz::PrimitiveComxyPz(MultiContactStateManager* mcsManager,
				   MultiContactStatePrimitiveGroup* mcsGroup,
				   MultiContactStatePrimitiveLoaderBase* loader,
				   MultiContactStateTransition* parentState,
				   const std::string& name, const std::string& guid) :

  MultiContactStatePrimitiveBase(mcsManager, mcsGroup, loader, parentState, name, guid),

  m_com_posInterpolationTiming({0.0, 0.0}),
  m_com_desFinalLinearVelocity(Vector3::Zero()),
  
  m_body_posInterpolationTiming({0.0, 0.0}),
  m_body_desFinalLinearVelocity(Vector3::Zero()),

  m_zmp_posInterpolationTiming({0.0, 0.0}),
  m_zmp_desFinalLinearVelocity(Vector3::Zero())
{}

void PrimitiveComxyPz::onEntry(void)
{
  if (m_com_interpolator && m_com_desPositions.size() > 0) {
    m_com_interpolator->configure_timing(m_com_posInterpolationTiming.t0,
                                         m_com_posInterpolationTiming.tf);
    m_com_interpolator->set_positions(m_com_desPositions);
    m_com_interpolator->set_final_linear_velocity(m_com_desFinalLinearVelocity);
  }
  
  if (m_bodyPosition_interpolator && m_body_desPositions.size() > 0) {
    m_bodyPosition_interpolator->configure_timing(m_body_posInterpolationTiming.t0,
                                                  m_body_posInterpolationTiming.tf);
    m_bodyPosition_interpolator->set_positions(m_body_desPositions);
    m_bodyPosition_interpolator->set_final_linear_velocity(m_body_desFinalLinearVelocity);
  }

  if (m_zmp_interpolator && m_zmp_desPositions.size() > 0) {
    
    m_zmp_interpolator->configure_timing(m_zmp_posInterpolationTiming.t0,
                                         m_zmp_posInterpolationTiming.tf);
    m_zmp_interpolator->set_positions(m_zmp_desPositions);
    m_zmp_interpolator->set_final_linear_velocity(m_zmp_desFinalLinearVelocity);
  }
}

void PrimitiveComxyPz::onExit(void)
{
  m_com_posInterpolationTiming = {0.0, 0.0};
  m_com_desPositions.clear();
  m_com_desFinalLinearVelocity.setZero();

  m_body_posInterpolationTiming = {0.0, 0.0};
  m_body_desPositions.clear();
  m_body_desFinalLinearVelocity.setZero();

  m_zmp_posInterpolationTiming = {0.0, 0.0};
  m_zmp_desPositions.clear();
  m_zmp_desFinalLinearVelocity.setZero();
}

bool PrimitiveComxyPz::check(int n_time, const std::bitset<NUM_MULTI_CONTACT_STATE_STOP_ACTIONS>& signal_state)
{
  return (m_com_interpolator->is_done() &&
          m_bodyPosition_interpolator->is_done() &&
          m_zmp_interpolator->is_done());
}
