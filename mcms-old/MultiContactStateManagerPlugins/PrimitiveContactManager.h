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

#ifndef __PRIMITIVE_CONTACT_MANAGER_H__
#define __PRIMITIVE_CONTACT_MANAGER_H__

#include "../MultiContactStatePrimitiveBase.h"

namespace multi_contact_motion_solver {
  class ContactConstraintManager;
}

namespace multi_contact_state_manager {

  class PrimitiveContactManager : public MultiContactStatePrimitiveBase {

  public:

    PrimitiveContactManager(MultiContactStateManager* mcsManager,
			     MultiContactStatePrimitiveGroup* mcsGroup,
			     MultiContactStatePrimitiveLoaderBase* loader,
			     MultiContactStateTransition* parentState,
			     const std::string& name, const std::string& guid);
    ~PrimitiveContactManager(void) {}

    void registerContactManager(multi_contact_motion_solver::ContactConstraintManager* manager)
    { m_contact_manager = manager; }

    void setRobotSurface(std::string surface)
    { m_robotSurface = surface; }

    void setEnvSurface(std::string surface)
    { m_envSurface = surface; }

    void setConstrained(bool constrained)
    { m_constrained = constrained; }
    
    void setStiffnessGain(double gain)
    { m_stiffnessGain = gain; }

    void setStiffnessMask(hrp::dvector6 mask)
    { m_stiffnessMask = mask; }

    void setDampingGain(double gain)
    { m_dampingGain = gain; }

    void setDampingMask(hrp::dvector6 mask)
    { m_dampingMask = mask; }

  private:

    multi_contact_motion_solver::ContactConstraintManager* m_contact_manager;

    std::string m_robotSurface;
    std::string m_envSurface;

    bool m_constrained;

    double m_stiffnessGain, m_dampingGain;
    hrp::dvector6 m_stiffnessMask, m_dampingMask;

    // Primitive Behavior Related
  
    void onEntry(void);
    void onExit(void) {}
    bool check(int n_time, const std::bitset<NUM_MULTI_CONTACT_STATE_STOP_ACTIONS>& signal_state) { return true; }
  };

}

#endif // __PRIMITIVE_CONTACT_MANAGER_H__
