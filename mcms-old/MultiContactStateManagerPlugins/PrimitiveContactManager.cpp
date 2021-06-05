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
#include <MultiContactMotionSolver/Constraints/ContactConstraintManager.h>
#include "PrimitiveContactManager.h"

using namespace hrp;
using namespace multi_contact_state_manager;
using namespace multi_contact_motion_solver;

PrimitiveContactManager::PrimitiveContactManager(MultiContactStateManager* mcsManager,
						   MultiContactStatePrimitiveGroup* mcsGroup,
						   MultiContactStatePrimitiveLoaderBase* loader,
						   MultiContactStateTransition* parentState,
						   const std::string& name, const std::string& guid) :

  MultiContactStatePrimitiveBase(mcsManager, mcsGroup, loader, parentState, name, guid),

  m_robotSurface(""),
  m_envSurface(""),

  m_constrained(false),

  m_stiffnessGain(0.0),
  m_dampingGain(200.0),

  m_stiffnessMask(dvector6::Ones()),
  m_dampingMask(dvector6::Ones())
{}

void PrimitiveContactManager::onEntry(void)
{
  if (m_contact_manager)
    m_contact_manager->set_contact_between_surfaces(m_robotSurface, m_envSurface, m_constrained,
						    m_stiffnessMask * m_stiffnessGain,
						    m_dampingMask * m_dampingGain);
}
