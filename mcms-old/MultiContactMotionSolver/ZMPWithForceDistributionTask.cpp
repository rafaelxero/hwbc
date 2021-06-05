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

#include "ZMPWithForceDistributionTask.h"

using namespace hrp;
using namespace common_interface;
using namespace multi_contact_motion_solver;

ZMPWithForceDistributionTask::ZMPWithForceDistributionTask(TaskExecutionManager* parentManager,
                                                           const char* name,
                                                           std::list<std::string>* cmd_init)
: ForceDistributionTaskBase(parentManager, name, cmd_init),
    
  // Shared InPort Data (Initializations)
  
  m_zmp_DesIn(NULL),
  m_zmp_DesIn_share(m_zmp_DesIn),

  m_zmp_ModIn(NULL),
  m_zmp_ModIn_share(m_zmp_ModIn),

  // Shared OutPort Data (Initializations)

  m_zmp_DesOut(Vector3::Zero()),
  m_zmp_DesOut_share(m_zmp_DesOut),

  m_zmp_ModOut(Vector3::Zero()),
  m_zmp_ModOut_share(m_zmp_ModOut),
  
  m_zmp_RefOut(Vector3::Zero()),
  m_zmp_RefOut_share(m_zmp_RefOut),

  m_zmp_CalOut(Vector3::Zero()),
  m_zmp_CalOut_share(m_zmp_CalOut),

  m_zmp_ErrOut(Vector3::Zero()),
  m_zmp_ErrOut_share(m_zmp_ErrOut),

  m_moment_ErrOut(Vector3::Zero()),
  m_moment_ErrOut_share(m_moment_ErrOut),
  
  // Task related

  m_zmp_default(Vector3::Zero())
{
  // Shared InPort Data (Registrations)

  registerInPort("zmp_DesIn", (sharedFuncPtr) &ZMPWithForceDistributionTask::zmp_DesIn);
  registerInPort("zmp_ModIn", (sharedFuncPtr) &ZMPWithForceDistributionTask::zmp_ModIn);

  // Shared OutPort Data (Registrations)

  registerOutPort("zmp_DesOut", (sharedFuncPtr) &ZMPWithForceDistributionTask::zmp_DesOut);
  registerOutPort("zmp_ModOut", (sharedFuncPtr) &ZMPWithForceDistributionTask::zmp_ModOut);
  registerOutPort("zmp_RefOut", (sharedFuncPtr) &ZMPWithForceDistributionTask::zmp_RefOut);

  registerOutPort("zmp_CalOut", (sharedFuncPtr) &ZMPWithForceDistributionTask::zmp_CalOut);
  registerOutPort("zmp_ErrOut", (sharedFuncPtr) &ZMPWithForceDistributionTask::zmp_ErrOut);
  registerOutPort("moment_ErrOut", (sharedFuncPtr) &ZMPWithForceDistributionTask::moment_ErrOut);
  
  // Method Functions

  registerMethodFunction(":set-zmp",
                         (methodFuncPtr) &ZMPWithForceDistributionTask::cmd_set_zmp);
  registerMethodFunction(":set-dim-weight",
                         (methodFuncPtr) &ZMPWithForceDistributionTask::cmd_set_dim_weight);
  
  initialCommands(cmd_init);

  // Task Related

  m_fdist_task = new tasks::qp::ZMPWithForceDistributionTask(m_motion_solver->robots().mbs(),
                                                             m_motion_solver->robots().robotIndex(),
                                                             Vector3::Zero(), m_weight_default);
}

bool ZMPWithForceDistributionTask::onExecute(void)
{
  tasks::qp::ZMPWithForceDistributionTask * zmpFdistTask = dynamic_cast<tasks::qp::ZMPWithForceDistributionTask*>(m_fdist_task);
  
  if (zmpFdistTask) {

    Vector3 zmp_RefIn(Vector3::Zero());
    
    if (zmp_DesIn()->isConnected())
      m_zmp_DesOut = *m_zmp_DesIn;
    
    zmp_RefIn = m_zmp_DesOut;
    
    if (zmp_ModIn()->isConnected())
      m_zmp_ModOut = *m_zmp_ModIn;
    else
      m_zmp_ModOut = Vector3::Zero();

    zmp_RefIn += m_zmp_ModOut;

    zmpFdistTask->zmp(zmp_RefIn);
    
    m_zmp_RefOut = zmpFdistTask->zmp();
    
    Vector3 totalForce = zmpFdistTask->totalForce();
    Vector3 totalMomentZMP = zmpFdistTask->totalMomentZMP();
    
    m_zmp_CalOut = {-totalMomentZMP(Y) / totalForce(Z),
                    totalMomentZMP(X) / totalForce(Z),
                    zmpFdistTask->zmp()(Z)};
    
    m_zmp_ErrOut = m_zmp_RefOut - m_zmp_CalOut;

    if (totalForce.norm() != 0)  // Rafa's patch
      m_moment_ErrOut = m_zmp_ErrOut.cross(totalForce);
    else
      m_moment_ErrOut.setZero();
    
    return true;
  }
  else {
    std::cerr << getTaskName() << " : invalid task" << std::endl;
    return false;
  }
}

bool ZMPWithForceDistributionTask::
cmd_set_zmp(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  Vector3 zmp;
  i_strm >> zmp[0] >> zmp[1] >> zmp[2];

  tasks::qp::ZMPWithForceDistributionTask * zmpFdistTask = dynamic_cast<tasks::qp::ZMPWithForceDistributionTask*>(m_fdist_task);

  if (zmpFdistTask) {
    zmpFdistTask->zmp(zmp);
    o_strm << getTaskName() << " : set the zmp to: " << zmpFdistTask->zmp().transpose() << std::endl;
    return true;
  }
  else {
    std::cerr << getTaskName() << " : invalid task" << std::endl;
    return false;
  }
}

bool ZMPWithForceDistributionTask::
cmd_set_dim_weight(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  Vector3 w_vec;
  i_strm >> w_vec[0] >> w_vec[1] >> w_vec[2];

   tasks::qp::ZMPWithForceDistributionTask * zmpFdistTask = dynamic_cast<tasks::qp::ZMPWithForceDistributionTask*>(m_fdist_task);

  if (zmpFdistTask) {
    zmpFdistTask->dimWeight(w_vec);
    o_strm << "set the dim weight of " << getTaskName() << " as: "
           << zmpFdistTask->dimWeight().transpose() << std::endl;
    return true;
  }
  else {
    std::cerr << getTaskName() << " : invalid task" << std::endl;
    return false;
  }
}
