// -*- mode: c++; indent-tabs-mode: nil; tab-width: 2; c-basic-offset: 2; -*-
/*
 * Copyright (c) 2017
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

#ifndef __CONTACT_CONSTRAINT_H__
#define __CONTACT_CONSTRAINT_H__

#include <HumanoidMotionControlCore/HumanoidMotionControlCore.h>
#include <Interface/TaskExecutionHandler.h>
#include <MultiContactMotionSolver/MultiContactMotionSolver.h>
#include "ContactConstraintManager.h"

namespace humanoid_motion_control_core {
  class HumanoidMotionControlCore;
}

namespace multi_contact_motion_solver {

  class ContactConstraint : public common_interface::TaskExecutionHandler {

  public:

    ContactConstraint(common_interface::TaskExecutionManager* parentManager,
                      const char* name = "contact-constraint",
                      std::list<std::string>* cmd_init = NULL);

    bool isReadyToActivate(void);
    bool isReadyToDeactivate(void);
    bool onActivate(void);
    bool onExecute(void);

  protected:

    humanoid_motion_control_core::HumanoidMotionControlCore* hmc;
    MultiContactMotionSolver* m_motion_solver;
    ContactConstraintManager* m_contact_constraint_manager;

  private:

    bool m_contactModified;
    std::string m_robotSurface;
    std::string m_envSurface;

    void updateContact();

    /** Select the surface of the robot (robot-surf-name) that will be constrained.
     *
     *  Usage:
     *  ":select-robot-surface robot-surf-name"
     */
    bool cmd_select_robot_surface(std::istringstream& i_strm, std::ostringstream& o_strm);

  /** Select the surface of the environment (env-surf-name) to which the surface of the robot will be constrained to.
     *
     *  Usage:
     *  ":select-environment-surface env-surf-name"
     */
    bool cmd_select_environment_surface(std::istringstream& i_strm, std::ostringstream& o_strm);
  };
  
}

#endif // __CONTACT_CONSTRAINT_H__
