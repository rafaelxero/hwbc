// -*- mode: c++; indent-tabs-mode: nil; tab-width: 2; c-basic-offset: 2; -*-
/*
 * Copyright (c) 2019
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

#include "NullSpaceAdmittanceTask.h"
#include <Tasks/QPTasks.h>

using namespace hrp;
using namespace common_interface;
using namespace multi_contact_motion_solver;

NullSpaceAdmittanceTask::NullSpaceAdmittanceTask(TaskExecutionManager* parentManager,
						 const char* name,
						 std::list<std::string>* cmd_init) :

  AdmittanceTaskBase(parentManager, name, cmd_init),
  m_admittance_manager(dynamic_cast<AdmittanceManager*>(parentManager)),
  
  // Shared InPort Data (Initializations)

  m_fdistRatio_DesIn(NULL),
  m_fdistRatio_DesIn_share(m_fdistRatio_DesIn)
{
  // Shared InPort Data (Registrations)

  registerInPort("fdistRatio_DesIn", (sharedFuncPtr) &NullSpaceAdmittanceTask::fdistRatio_DesIn);

  // Method Functions
  
  registerMethodFunction(":set-fdist-ratio",
                         (methodFuncPtr) &NullSpaceAdmittanceTask::cmd_set_fdist_ratio);
  
  initialCommands(cmd_init);

  // Task Related

  m_task = new tasks::qp::NullSpaceAdmittanceTask(m_motion_solver->robots().mbs(),
						  m_motion_solver->robots().robotIndex(),
						  m_targetLink, m_bodyPoint, m_dt,
						  m_force_stiffness_default, m_force_damping_default,
						  m_moment_stiffness_default, m_moment_damping_default,
						  m_weight_default);

  if (m_admittance_manager) {
    m_admittance_manager->registerAdmittanceTask(this);
    std::cout << "register " << getTaskName() << " to " << m_admittance_manager->getTaskName() << std::endl;
  }
}

bool NullSpaceAdmittanceTask::onActivate(void)
{
  if (!m_admittance_manager) {
    std::cout << "ERROR!!!: AdmittanceManager is not found in " << getTaskName() << std::endl;
    return false;
  }

  return AdmittanceTaskBase::onActivate();
}

bool NullSpaceAdmittanceTask::onExecute(void)
{
  if (m_estimated_beforehand) {

    m_estimated_beforehand = false;
    
    tasks::qp::NullSpaceAdmittanceTask* admittance_task = dynamic_cast<tasks::qp::NullSpaceAdmittanceTask*>(m_task);    
    
    if (admittance_task) {
      
      admittance_task->measuredWrenches(m_admittance_manager->measuredWrenches());

      if (fdistRatio_DesIn()->isConnected())
        admittance_task->fdistRatio(*m_fdistRatio_DesIn);
      
      return true;
    }
    else {
      std::cerr << getTaskName() << " : invalid task" << std::endl;
      return false;
    }
  }
  else {
    std::cerr << getTaskName() << " : the estimated wrench was not computed beforehand" << std::endl;
    return false;
  }
}

bool NullSpaceAdmittanceTask::
cmd_set_fdist_ratio(std::istringstream& i_strm, std::ostringstream& o_strm)
{
}
