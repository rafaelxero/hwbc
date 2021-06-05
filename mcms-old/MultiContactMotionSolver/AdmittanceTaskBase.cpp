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

#include "AdmittanceTaskBase.h"
#include <Tasks/QPTasks.h>

using namespace hrp;
using namespace common_interface;
using namespace multi_contact_motion_solver;

AdmittanceTaskBase::AdmittanceTaskBase(TaskExecutionManager* parentManager,
                                       const char* name,
                                       std::list<std::string>* cmd_init) :
  
  WrenchSpaceTask(parentManager, name, cmd_init),

  // Shared OutPort Data (Initializations)

  m_force_CalOut(Vector3::Zero()),
  m_force_CalOut_share(m_force_CalOut),

  m_moment_CalOut(Vector3::Zero()),
  m_moment_CalOut_share(m_moment_CalOut),
  
  // Task Related

  m_sensorLink(""),
  
  m_force_stiffness_default(1e-1),
  m_force_damping_default(1e-4),
  m_moment_stiffness_default(2),
  m_moment_damping_default(3e-2),

  m_measuredWrench(sva::ForceVecd::Zero()),
  
  m_force_LPFilter(0.3),
  m_moment_LPFilter(0.3),

  m_estimated_beforehand(false)
{
  // Shared OutPort Data (Registrations)
  
  registerOutPort("force_HatOut",  (sharedFuncPtr) &AdmittanceTaskBase::force_CalOut);
  registerOutPort("moment_HatOut", (sharedFuncPtr) &AdmittanceTaskBase::moment_CalOut);
  
  // Method Functions

  registerMethodFunction(":set-force-pd-gains",
                         (methodFuncPtr) &AdmittanceTaskBase::cmd_set_force_pd_gains);
  registerMethodFunction(":set-moment-pd-gains",
                         (methodFuncPtr) &AdmittanceTaskBase::cmd_set_moment_pd_gains);
  
  registerMethodFunction(":set-dim-weight",
                         (methodFuncPtr) &AdmittanceTaskBase::cmd_set_dim_weight);
  registerMethodFunction(":treat-settings-as-local",
                         (methodFuncPtr) &AdmittanceTaskBase::cmd_treat_settings_as_local);

  // Task Related

  m_force_LPFilter.reset(3);
  m_moment_LPFilter.reset(3);
}

void AdmittanceTaskBase::estimateMeasuredWrench()
{
  const mc_rbdyn::ForceSensor & sensor = m_motion_solver->robot().bodyForceSensor(m_sensorLink);

  int SensorLinkId = m_motion_solver->robot().mb().bodyIndexByName(m_sensorLink);
  std::string SensorJointName = m_motion_solver->robot().mb().joint(SensorLinkId).name();
    
  sva::PTransformd SensorRelT = sensor.X_p_f();
  Matrix33 SensorLinkRot = m_motion_solver->robot().mbc().bodyPosW[SensorLinkId].rotation().transpose();
  Matrix33 SensorRot = SensorLinkRot * SensorRelT.rotation().transpose();
  Vector3  SensorPos = m_motion_solver->robot().mbc().bodyPosW[SensorLinkId].translation() + SensorLinkRot * SensorRelT.translation();

  int TargetLinkId = m_motion_solver->robot().mb().bodyIndexByName(m_targetLink);
  Vector3  TargetLinkPos = m_motion_solver->robot().mbc().bodyPosW[TargetLinkId].translation();
  Matrix33 TargetLinkRot = m_motion_solver->robot().mbc().bodyPosW[TargetLinkId].rotation().transpose();
  Vector3  TargetLinkRefPos = TargetLinkPos + TargetLinkRot * m_bodyPoint;

  int index = m_motion_solver->body()->link(SensorJointName)->index;
  Vector3 TargetWeight = {0.0, 0.0, m_motion_solver->linkInr()[index].Mass * -9.81};
  Vector3 TargetCom = m_motion_solver->linkInr()[index].Com;
    
  Vector3 SensorForce = SensorRot * sensor.force();
  Vector3 SensorMoment = SensorRot * sensor.couple();

  Vector3 force  = SensorForce - TargetWeight;
  Vector3 moment = SensorMoment - (SensorPos - TargetLinkRefPos).cross(SensorForce) - (TargetCom - TargetLinkRefPos).cross(TargetWeight);

  m_measuredWrench.force() = m_force_LPFilter.LPF(force);
  m_measuredWrench.couple() = m_moment_LPFilter.LPF(moment);

  std::cout << "Rafa, in AdmittanceTaskBase::estimateMeasuredWrench ("
            << getTaskName() << "), m_measuredWrench.force() = "
            << m_measuredWrench.force().transpose() << std::endl;
  std::cout << "Rafa, in AdmittanceTaskBase::estimateMeasuredWrench ("
            << getTaskName() << "), m_measuredWrench.couple() = "
            << m_measuredWrench.couple().transpose() << std::endl;
}

bool AdmittanceTaskBase::
cmd_target_link(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  bool success = WrenchSpaceTask::cmd_target_link(i_strm, o_strm);
  
  if (success) {

    if (m_motion_solver->robot().bodyHasForceSensor(m_targetLink))

      m_sensorLink = m_targetLink;
    
    else {

      int targetLinkId = m_motion_solver->robot().mb().bodyIndexByName(m_targetLink);
      int parentLinkId = m_motion_solver->robot().mb().parent(targetLinkId);
      std::string parentLink = m_motion_solver->robot().mb().body(parentLinkId).name();

      if (m_motion_solver->robot().bodyHasForceSensor(parentLink))
        m_sensorLink = parentLink;
      else {
        std::cerr << getTaskName() << " : there is no F/T in the vecinity of " << m_targetLink << std::endl;
        return false;
      }
    }

    o_strm << "configure " << m_sensorLink << " as the F/T sensor link of " << getTaskName()
           << std::endl;
    
    return true;
  }

  else

    return false;
}

bool AdmittanceTaskBase::
cmd_set_force_pd_gains(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  double kp, kd;
  i_strm >> kp >> kd;

  tasks::qp::AdmittanceTaskCommon* admittance_task = dynamic_cast<tasks::qp::AdmittanceTaskCommon*>(m_task);

  if (admittance_task) {
    
    admittance_task->setForceGains(kp, kd);
    
    o_strm << "set the stiffness and damping (PD gains) for the force of " << getTaskName() << " as: " << kp << " and " << kd <<  std::endl;
    
    return true;
  }
  else {
    
    std::cerr << getTaskName() << " : cannot set the stiffness and damping (PD gains) for the force" << std::endl;
    
    return false;
  }
}

bool AdmittanceTaskBase::
cmd_set_moment_pd_gains(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  double kp, kd;
  i_strm >> kp >> kd;

  tasks::qp::AdmittanceTaskCommon* admittance_task = dynamic_cast<tasks::qp::AdmittanceTaskCommon*>(m_task);

  if (admittance_task) {
    
    admittance_task->setCoupleGains(kp, kd);
  
    o_strm << "set the stiffness and damping  (PD gains) for the force of " << getTaskName() << " as: " << kp << " and " << kd <<  std::endl;

    return true;
  }
  else {

    std::cerr << getTaskName() << " : cannot set the stiffness and damping (PD gains) for the moment" << std::endl;
    
    return false;
  }
}

bool AdmittanceTaskBase::
cmd_set_dim_weight(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  Vector3 fw_vec;
  Vector3 nw_vec;

  for (size_t i = 0; i < fw_vec.size(); i++)
    if (i_strm.good())
      i_strm >> fw_vec[i];
    else {
      std::cerr << getTaskName() << " : invalid dimension for the weight vector (dimWeight) related to the force" << std::endl;
      return false;
    }

  for (size_t i = 0; i < nw_vec.size(); i++)
    if (i_strm.good())
      i_strm >> nw_vec[i];
    else {
      std::cerr << getTaskName() << " : invalid dimension for the weight vector (dimWeight) related to the moment" << std::endl;
      return false;
    }

  dvector6 w_vec;
  w_vec << nw_vec, fw_vec;

  tasks::qp::AdmittanceTaskCommon* admittance_task = dynamic_cast<tasks::qp::AdmittanceTaskCommon*>(m_task);

  if (admittance_task) {
    
    admittance_task->dimWeight(m_motion_solver->robots().mbcs(), w_vec);
  
    o_strm << "set the dim weight of " << getTaskName() << " as: ";
    for (size_t i = 0; i < w_vec.size(); i++)
      o_strm << w_vec[i] << " ";
    o_strm << std::endl;
  
    return true;
  }
  else { 
    std::cerr << getTaskName() << " : cannot set dimWeight" << std::endl;
    return false;
  }
}

bool AdmittanceTaskBase::
cmd_treat_settings_as_local(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  bool local;

  i_strm >> local;

  tasks::qp::AdmittanceTaskCommon* admittance_task = dynamic_cast<tasks::qp::AdmittanceTaskCommon*>(m_task);

  if (admittance_task) {
  
    admittance_task->treatSettingsAsLocal(local);

    o_strm << getTaskName() << " : treat the desired settings as given with respect to the ";
    if (local)
      o_strm << "local";
    else
      o_strm << "global";
    o_strm << " frame" << std::endl;

    return true;
  }
  else {
    std::cerr << getTaskName() << " : cannot set the treatment of the settings" << std::endl;
    return false;
  }
}
