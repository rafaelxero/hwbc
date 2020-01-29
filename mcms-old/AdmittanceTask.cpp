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

#include "AdmittanceTask.h"
#include <Tasks/QPTasks.h>

using namespace hrp;
using namespace common_interface;
using namespace multi_contact_motion_solver;

AdmittanceTask::AdmittanceTask(TaskExecutionManager* parentManager,
                               const char* name,
                               std::list<std::string>* cmd_init) :

  AdmittanceTaskBase(parentManager, name, cmd_init)
{
  initialCommands(cmd_init);
  
  // Task Related
  
  m_task = new tasks::qp::AdmittanceTask(m_motion_solver->robots().mbs(),
                                         m_motion_solver->robots().robotIndex(),
                                         m_targetLink, m_bodyPoint, m_dt,
                                         m_force_stiffness_default, m_force_damping_default,
                                         m_moment_stiffness_default, m_moment_damping_default,
                                         m_weight_default);
}

bool AdmittanceTask::onExecute(void)
{
  tasks::qp::AdmittanceTask* admittance_task = dynamic_cast<tasks::qp::AdmittanceTask*>(m_task);
  
  if (admittance_task) {

    if (!m_estimated_beforehand)
      estimateMeasuredWrench();
    
    admittance_task->measuredWrench(m_measuredWrench);

    m_estimated_beforehand = false;
    return true;
  }
  else {
    std::cerr << getTaskName() << " : invalid task" << std::endl;
    return false;
  }
}
