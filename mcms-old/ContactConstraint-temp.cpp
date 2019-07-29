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

#include "ContactConstraint.h"

using namespace hrp;
using namespace common_interface;
using namespace humanoid_motion_control_core;
using namespace multi_contact_motion_solver;

ContactConstraint::ContactConstraint(TaskExecutionManager* parentManager,
                                     const char* name,
                                     std::list<std::string>* cmd_init) :

  TaskExecutionHandler(parentManager, name, cmd_init),
  hmc(dynamic_cast<HumanoidMotionControlCore*>(findRootTaskExecutionManager(parentManager))),
  m_motion_solver(hmc->findTaskExecution<MultiContactMotionSolver*>()),
  m_contact_constraint_manager(hmc->findTaskExecution<ContactConstraintManager*>()),
  
  // Constraint Related

  m_contactModified(true),
  m_robotSurface(""),
  m_envSurface("AllGround")
{
  // Methods Functions

  registerMethodFunction(":select-robot-surface",
                         (methodFuncPtr) &ContactConstraint::cmd_select_robot_surface);
  registerMethodFunction(":select-environment-surface",
                         (methodFuncPtr) &ContactConstraint::cmd_select_environment_surface);  
  
  initialCommands(cmd_init);
}

bool ContactConstraint::isReadyToActivate(void)
{
  updateContact();
  m_contactModified = false;

  return TaskExecutionHandler::isReadyToActivate();
}

bool ContactConstraint::isReadyToDeactivate(void)
{
  m_contact_constraint_manager->removeContact(getTaskName());
  m_contactModified = false;
  
  return TaskExecutionHandler::isReadyToDeactivate();
}

bool ContactConstraint::onActivate(void)
{
  if (!m_motion_solver) {
    std::cout << "ERROR!!!: MultiContactMotionSolver is not found in " << getTaskName() << std::endl;
    return false;
  }
  
  if (!m_contact_constraint_manager) {
    std::cout << "ERROR!!!: ContactConstraintManager is not found in " << getTaskName() << std::endl;
    return false;
  }
  
  return TaskExecutionHandler::onActivate();
}

bool ContactConstraint::onExecute(void)
{
  if (m_contactModified) {
    updateContact();
    m_contactModified = false;
  }
}

void ContactConstraint::updateContact()
{
  mc_rbdyn::Contact contact = {m_motion_solver->robots(), 0, 1, m_robotSurface, m_envSurface};
  m_contact_constraint_manager->updateContact(getTaskName(), contact);
}

bool ContactConstraint::
cmd_select_robot_surface(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  if (m_robotSurface == "") {

    std::string robotSurface;
    i_strm >> robotSurface;

    if (m_motion_solver->robots().robot(0).hasSurface(robotSurface)) {
      m_robotSurface = robotSurface;
      o_strm << "constrain " << m_robotSurface << " to " << m_envSurface << std::endl;
    }
    else
      std::cerr << getTaskName() << " : invalid robot surface" << std::endl;
  }
  else
    o_strm << "WARNING! cannot modify the robot surface the corresponding Task Space Constraint has been created" << std::endl;

  m_contactModified = true;
}

bool ContactConstraint::
cmd_select_environment_surface(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  std::string envSurface;
  i_strm >> envSurface;
  
  if (m_motion_solver->robots().robot(1).hasSurface(envSurface)) {
    m_envSurface = envSurface;
    o_strm << "constrain " << m_robotSurface << " to " << m_envSurface << std::endl;
  }
  else
    std::cerr << getTaskName() << " : invalid environment surface" << std::endl;

  m_contactModified = true;
}
