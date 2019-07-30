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

#include "PoseTask.h"
// #include <MotionGenerator/WalkingStateManager/WalkingStateManager.h>  // Rafa
// #include <MotionGenerator/WalkingStateManager/WalkingState.h>  // Rafa

using namespace hrp;
using namespace common_interface;
using namespace multi_contact_motion_solver;
// using namespace walking_state_manager;  // Rafa

PoseTask::PoseTask(TaskExecutionManager* parentManager,
                   const char* name,
                   std::list<std::string>* cmd_init) :
  
  CartesianSpaceTask(parentManager, name, cmd_init),

  // Shared InPort Data (Initializations)
  
  m_linkState_DesIn(NULL),
  m_linkState_DesIn_share(m_linkState_DesIn),

  m_linkStateLin_DesIn(NULL),
  m_linkStateLin_DesIn_share(m_linkStateLin_DesIn),
  
  m_linkStateAng_DesIn(NULL),
  m_linkStateAng_DesIn_share(m_linkStateAng_DesIn),

  m_linkPose_DesIn(NULL),
  m_linkPose_DesIn_share(m_linkPose_DesIn),
  
  m_linkP_DesIn(NULL),
  m_linkP_DesIn_share(m_linkP_DesIn),
  
  m_linkR_DesIn(NULL),
  m_linkR_DesIn_share(m_linkR_DesIn),
  
  // Shared OutPort Data (Initializations)

  m_linkState_DesOut(),
  m_linkState_DesOut_share(m_linkState_DesOut),
  m_linkStateLin_DesOut_share(m_linkState_DesOut),
  m_linkStateAng_DesOut_share(m_linkState_DesOut),
  m_linkPose_DesOut_share(m_linkState_DesOut),
  m_linkP_DesOut_share(m_linkState_DesOut.P),
  m_linkR_DesOut_share(m_linkState_DesOut.R),
  
  m_linkState_HatOut(),
  m_linkState_HatOut_share(m_linkState_HatOut),
  m_linkStateLin_HatOut_share(m_linkState_HatOut),
  m_linkStateAng_HatOut_share(m_linkState_HatOut),
  
  m_linkState_ErrOut(),
  m_linkState_ErrOut_share(m_linkState_ErrOut),
  m_linkStateLin_ErrOut_share(m_linkState_ErrOut),
  m_linkStateAng_ErrOut_share(m_linkState_ErrOut),

  m_linkState_RefOut(),
  m_linkState_RefOut_share(m_linkState_RefOut),
  m_linkStateLin_RefOut_share(m_linkState_RefOut),
  m_linkStateAng_RefOut_share(m_linkState_RefOut),
  m_linkPose_RefOut_share(m_linkState_RefOut),
  m_linkP_RefOut_share(m_linkState_RefOut.P),
  m_linkR_RefOut_share(m_linkState_RefOut.R)
{
  // Shared InPort Data (Registrations)

  registerInPort("linkState_DesIn",      (sharedFuncPtr) &PoseTask::linkState_DesIn);
  registerInPort("linkStateLin_DesIn",   (sharedFuncPtr) &PoseTask::linkStateLin_DesIn);
  registerInPort("linkStateAng_DesIn",   (sharedFuncPtr) &PoseTask::linkStateAng_DesIn);

  registerInPort("linkPose_DesIn",       (sharedFuncPtr) &PoseTask::linkPose_DesIn);
  registerInPort("linkP_DesIn",          (sharedFuncPtr) &PoseTask::linkP_DesIn);
  registerInPort("linkR_DesIn",          (sharedFuncPtr) &PoseTask::linkR_DesIn);
  
  // Shared OutPort Data (Registrations)

  registerOutPort("linkState_DesOut",    (sharedFuncPtr) &PoseTask::linkState_DesOut);
  registerOutPort("linkStateLin_DesOut", (sharedFuncPtr) &PoseTask::linkStateLin_DesOut);
  registerOutPort("linkStateAng_DesOut", (sharedFuncPtr) &PoseTask::linkStateAng_DesOut);
  registerOutPort("linkPose_DesOut",     (sharedFuncPtr) &PoseTask::linkPose_DesOut);
  registerOutPort("linkP_DesOut",        (sharedFuncPtr) &PoseTask::linkP_DesOut);
  registerOutPort("linkR_DesOut",        (sharedFuncPtr) &PoseTask::linkR_DesOut);

  registerOutPort("linkState_HatOut",    (sharedFuncPtr) &PoseTask::linkState_HatOut);
  registerOutPort("linkStateLin_HatOut", (sharedFuncPtr) &PoseTask::linkStateLin_HatOut);
  registerOutPort("linkStateAng_HatOut", (sharedFuncPtr) &PoseTask::linkStateAng_HatOut);
  
  registerOutPort("linkState_ErrOut",    (sharedFuncPtr) &PoseTask::linkState_ErrOut);
  registerOutPort("linkStateLin_ErrOut", (sharedFuncPtr) &PoseTask::linkStateLin_ErrOut);
  registerOutPort("linkStateAng_ErrOut", (sharedFuncPtr) &PoseTask::linkStateAng_ErrOut);

  registerOutPort("linkState_RefOut",    (sharedFuncPtr) &PoseTask::linkState_RefOut);
  registerOutPort("linkStateLin_RefOut", (sharedFuncPtr) &PoseTask::linkStateLin_RefOut);
  registerOutPort("linkStateAng_RefOut", (sharedFuncPtr) &PoseTask::linkStateAng_RefOut);
  registerOutPort("linkPose_RefOut",     (sharedFuncPtr) &PoseTask::linkPose_RefOut);
  registerOutPort("linkP_RefOut",        (sharedFuncPtr) &PoseTask::linkP_RefOut);
  registerOutPort("linkR_RefOut",        (sharedFuncPtr) &PoseTask::linkR_RefOut);
  
  // Method Functions

  registerMethodFunction(":set-position",
                         (methodFuncPtr) &PoseTask::cmd_set_position);
  registerMethodFunction(":set-orientation",
                         (methodFuncPtr) &PoseTask::cmd_set_orientation);

  registerMethodFunction(":shift-position",
                         (methodFuncPtr) &PoseTask::cmd_shift_position);

  initialCommands(cmd_init);

  // Task Related

  m_task.push_back(new mc_tasks::PositionTask(m_targetLink, m_bodyPoint, m_motion_solver->robots(), m_motion_solver->robots().robotIndex()));
  m_task.push_back(new mc_tasks::OrientationTask(m_targetLink, m_motion_solver->robots(), m_motion_solver->robots().robotIndex()));
}

bool PoseTask::onExecute(void)
{
  // Rafa, the following code (wsm) is just for test
  /*
  WalkingStateManager* wsm = hmc->findTaskExecution<WalkingStateManager*>();

  if (wsm) {
    
    const WalkingState* wsm_ws = &wsm->WalkingPhase();

    if (isDoubleSupport(*wsm_ws)) {
      if (getTaskName() == "rfoot-pose-task" || getTaskName() == "lfoot-pose-task")
        for (size_t i = 0; i < m_task.size(); i++)
          m_task[i]->setGains(20, 100);
    }
    else if (isRightSupport(*wsm_ws)) {
      if (getTaskName() == "rfoot-pose-task")
        for (size_t i = 0; i < m_task.size(); i++)
          m_task[i]->setGains(20, 100);
      else if (getTaskName() == "lfoot-pose-task")
        for (size_t i = 0; i < m_task.size(); i++)
          m_task[i]->setGains(100, 20);
    }
    else if (isLeftSupport(*wsm_ws)) {
      if (getTaskName() == "rfoot-pose-task")
        for (size_t i = 0; i < m_task.size(); i++)
          m_task[i]->setGains(100, 20);
      else if (getTaskName() == "lfoot-pose-task")
        for (size_t i = 0; i < m_task.size(); i++)
          m_task[i]->setGains(20, 100);
    }
    else {
      if (getTaskName() == "rfoot-pose-task" || getTaskName() == "lfoot-pose-task")
        for (size_t i = 0; i < m_task.size(); i++)
          m_task[i]->setGains(100, 20);
    }
  }

  std::cout << "Rafa, in PoseTask::onExecute for " << getTaskName() << ", m_task[0]->stiffness() = " << m_task[0]->stiffness() << std::endl;
  */
  
  mc_tasks::PositionTask* posTask = positionTask();
  mc_tasks::OrientationTask* orTask = orientationTask();

  // if (getTaskName() == "body-pose-task") {
  //  std::cout << "Rafa, in PoseTask::onExecute, for " << getTaskName() << " m_linkState_DesIn_share.isConnected() = " << m_linkState_DesIn_share.isConnected() << std::endl;
  //  std::cout << "Rafa, in PoseTask::onExecute, for " << getTaskName() << " m_linkState_DesIn_share.TargetSharedData() = " << m_linkState_DesIn_share.TargetSharedData() << std::endl;
  //  std::cout << "Rafa, in PoseTask::onExecute, for " << getTaskName() << " m_linkState_DesIn = " << m_linkState_DesIn << std::endl;
  // }
  
  if (posTask && orTask) {

    // std::cout << "Rafa, in PoseTask::onExecute, posTask->bodyPoint() = " << posTask->bodyPoint().transpose() << std::endl;
    
    // if (m_linkState_DesIn) {
    if (linkState_DesIn()->isConnected()) {
      // if (getTaskName() == "body-pose-task")
      //   std::cout << "Rafa, in PoseTask::onExecute, for body-pose-task m_linkState_DesIn connected and about to change the desired value" << std::endl;
      posTask->position(m_linkState_DesIn->P);
      posTask->refVel(m_linkState_DesIn->V);
      posTask->refAccel(m_linkState_DesIn->Vdot);
    }
    // else if (m_linkStateLin_DesIn) {
    else if (linkStateLin_DesIn()->isConnected()) {
      // if (getTaskName() == "body-pose-task")
      //   std::cout << "Rafa, in PoseTask::onExecute, for body-pose-task m_linkStateLin_DesIn connected and about to change the desired value" << std::endl;
      posTask->position(m_linkStateLin_DesIn->P);
      posTask->refVel(m_linkStateLin_DesIn->V);
      posTask->refAccel(m_linkStateLin_DesIn->Vdot);
    }
    // else if (m_linkPose_DesIn) {
    else if (linkPose_DesIn()->isConnected()) {
      // if (getTaskName() == "body-pose-task")
      //   std::cout << "Rafa, in PoseTask::onExecute, for body-pose-task m_linkPose_DesIn connected and about to change the desired value" << std::endl;
      posTask->position(m_linkPose_DesIn->P);
      posTask->refVel(Vector3::Zero());
      posTask->refAccel(Vector3::Zero());
    }
    // else if (m_linkP_DesIn) {
    else if (linkP_DesIn()->isConnected()) {
      // if (getTaskName() == "body-pose-task")
      //   std::cout << "Rafa, in PoseTask::onExecute, for body-pose-task m_linkP_DesIn connected and about to change the desired value" << std::endl;
      posTask->position(*m_linkP_DesIn);
      posTask->refVel(Vector3::Zero());
      posTask->refAccel(Vector3::Zero());
    }

    // if (m_linkState_DesIn) {
    if (linkState_DesIn()->isConnected()) {
      orTask->orientation(m_linkState_DesIn->R);
      orTask->refVel(m_linkState_DesIn->W);
      orTask->refAccel(m_linkState_DesIn->Wdot);
    }
    // else if (m_linkStateAng_DesIn) {
    else if (linkStateAng_DesIn()->isConnected()) {
      orTask->orientation(m_linkStateAng_DesIn->R);
      orTask->refVel(m_linkStateAng_DesIn->W);
      orTask->refAccel(m_linkStateAng_DesIn->Wdot);
    }
    // else if (m_linkPose_DesIn) {
    else if (linkPose_DesIn()->isConnected()) {
      orTask->orientation(m_linkPose_DesIn->R);
      orTask->refVel(Vector3::Zero());
      orTask->refAccel(Vector3::Zero());
    }
    // else if (m_linkR_DesIn) {
    else if (linkR_DesIn()->isConnected()) {
      orTask->orientation(*m_linkR_DesIn);
      orTask->refVel(Vector3::Zero());
      orTask->refAccel(Vector3::Zero());
    }
    
    m_linkState_DesOut.P = posTask->position();
    m_linkState_DesOut.V = posTask->refVel();
    m_linkState_DesOut.Vdot = posTask->refAccel();

    m_linkState_DesOut.setR(orTask->orientation());
    m_linkState_DesOut.W = orTask->refVel();
    m_linkState_DesOut.Wdot = orTask->refAccel();
    
    int linkIndex = m_motion_solver->robot().bodyIndexByName(m_targetLink);

    // std::cout << "Rafa, in PoseTask::onExecute, m_targetLink = " << m_targetLink << std::endl;
    // std::cout << "Rafa, in PoseTask::onExecute, m_motion_solver->robot().mbc().bodyPosW[linkIndex].rotation() = " << std::endl << m_motion_solver->robot().mbc().bodyPosW[linkIndex].rotation() << std::endl;
    
    // Vector3 relHat_w = m_motion_solver->robot().mbc().bodyPosW[linkIndex].rotation() * m_bodyPoint;
    Vector3 relHat_w = m_motion_solver->robot().mbc().bodyPosW[linkIndex].rotation().transpose() * m_bodyPoint;
 
    m_linkState_HatOut.P = m_motion_solver->robot().mbc().bodyPosW[linkIndex].translation() + relHat_w;
    m_linkState_HatOut.V = m_motion_solver->robot().mbc().bodyVelW[linkIndex].linear() +
      m_motion_solver->robot().mbc().bodyVelW[linkIndex].angular().cross(relHat_w);

    // std::cout << "Rafa, in PoseTask::onExecute, m_motion_solver->robot().mbc().q[0][4-6] = " << m_motion_solver->robot().mbc().q[0][4] << " " << m_motion_solver->robot().mbc().q[0][5] << " " << m_motion_solver->robot().mbc().q[0][6] << std::endl;
    // std::cout << "Rafa, in PoseTask::onExecute for " << getTaskName() << ", m_linkState_HatOut.P = " << m_linkState_HatOut.P.transpose() << std::endl;
    // std::cout << "Rafa, in PoseTask::onExecute for " << getTaskName() << ", m_linkState_DesOut.P = " << m_linkState_DesOut.P.transpose() << std::endl;
    
    // m_linkState_HatOut.setR(m_motion_solver->robot().mbc().bodyPosW[linkIndex].rotation());
    m_linkState_HatOut.setR(m_motion_solver->robot().mbc().bodyPosW[linkIndex].rotation().transpose());
    m_linkState_HatOut.W = m_motion_solver->robot().mbc().bodyVelW[linkIndex].angular();

    Vector3 Om = orTask->eval();

    m_linkState_ErrOut.P = posTask->eval();
    m_linkState_ErrOut.V = posTask->refVel() - posTask->speed();

    m_linkState_ErrOut.setR(rodrigues(Om / Om.norm(), Om.norm()));
    m_linkState_ErrOut.W = orTask->refVel() - orTask->speed();

    // Vector3 relRef_w = m_motion_solver->solver().mbc_calc().bodyPosW[linkIndex].rotation() * m_bodyPoint;
    Vector3 relRef_w = m_motion_solver->solver().mbc_calc().bodyPosW[linkIndex].rotation().transpose() * m_bodyPoint;
    
    // Even if the following is meaningless, output it
    // if (m_motion_solver->torqueControlType() == torque_control::TorqueFeedbackTerm::PassivityPIDTerm) {
    
    m_linkState_RefOut.P = m_motion_solver->solver().mbc_calc().bodyPosW[linkIndex].translation() + relRef_w;
    m_linkState_RefOut.V = m_motion_solver->solver().mbc_calc().bodyVelW[linkIndex].linear() +
      m_motion_solver->solver().mbc_calc().bodyVelW[linkIndex].angular().cross(relRef_w);
    
    // m_linkState_RefOut.setR(m_motion_solver->solver().mbc_calc().bodyPosW[linkIndex].rotation());
    m_linkState_RefOut.setR(m_motion_solver->solver().mbc_calc().bodyPosW[linkIndex].rotation().transpose());
    m_linkState_RefOut.W = m_motion_solver->solver().mbc_calc().bodyVelW[linkIndex].angular();
    // }
    
    return true;
  }
  else {
    std::cerr << getTaskName() << " : invalid task" << std::endl;
    return false;
  }
}

mc_tasks::PositionTask* PoseTask::positionTask()
{
  mc_tasks::PositionTask* posTask;
  
  for (size_t i = 0; i < m_task.size(); i++) {
    posTask = dynamic_cast<mc_tasks::PositionTask*>(m_task[i]);
    if (posTask)
      break;
  }
  
  return posTask;
}

mc_tasks::OrientationTask* PoseTask::orientationTask()
{
  mc_tasks::OrientationTask* orTask;
  
  for (size_t i = 0; i < m_task.size(); i++) {
    orTask = dynamic_cast<mc_tasks::OrientationTask*>(m_task[i]);
    if (orTask)
      break;
  }
  
  return orTask;
}

bool PoseTask::
cmd_set_position(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  Vector3 p;
  i_strm >> p[0] >> p[1] >> p[2];

  mc_tasks::PositionTask* posTask = positionTask();

  if (posTask) {
    posTask->position(p);
    posTask->refVel(Vector3::Zero());
    posTask->refAccel(Vector3::Zero());
    o_strm << getTaskName() << " : set the position to: " << std::endl << posTask->position().transpose() << std::endl;
    return true;
  }
  else {
    std::cerr << getTaskName() << " : invalid task" << std::endl;
    return false;
  }
}

bool PoseTask::
cmd_set_orientation(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  Vector3 rpy;
  i_strm >> rpy[0] >> rpy[1] >> rpy[2];

  mc_tasks::OrientationTask* orTask = orientationTask();

  if (orTask) {
    orTask->orientation(rotFromRpy(rpy));
    orTask->refVel(Vector3::Zero());
    orTask->refAccel(Vector3::Zero());
    o_strm << "set the orientation to: " << std::endl << orTask->orientation() << std::endl;
    return true;
  }
  else {
    std::cerr << getTaskName() << " : invalid task" << std::endl;
    return false;
  }
}

bool PoseTask::
cmd_shift_position(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  Vector3 p_rel;
  i_strm >> p_rel[0] >> p_rel[1] >> p_rel[2];

  mc_tasks::PositionTask* posTask = positionTask();

  if (posTask) {
    posTask->move_position(p_rel);
    posTask->refVel(Vector3::Zero());
    posTask->refAccel(Vector3::Zero());
    o_strm << getTaskName() << " : set the position to: " << std::endl << posTask->position().transpose() << std::endl;
    return true;
  }
  else {
    std::cerr << getTaskName() << " : invalid task" << std::endl;
    return false;
  }
}
