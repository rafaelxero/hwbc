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

#ifndef __POSE_TASK_H__
#define __POSE_TASK_H__

#include "CartesianSpaceTask.h"
#include "PositionTask.h"
#include "OrientationTask.h"

namespace multi_contact_motion_solver {

  class PoseTask : public CartesianSpaceTask {

  public:

    PoseTask(common_interface::TaskExecutionManager* parentManager,
             const char* name = "pose-task",
             std::list<std::string>* cmd_init = NULL);

    bool onExecute(void);

    // Shared InPort Data (Methods)

    SharedDataBase* linkState_DesIn(void)     {return &m_linkState_DesIn_share;}
    SharedDataBase* linkStateLin_DesIn(void)  {return &m_linkStateLin_DesIn_share;}
    SharedDataBase* linkStateAng_DesIn(void)  {return &m_linkStateAng_DesIn_share;}

    SharedDataBase* linkPose_DesIn(void)      {return &m_linkPose_DesIn_share;}
    SharedDataBase* linkP_DesIn(void)         {return &m_linkP_DesIn_share;}
    SharedDataBase* linkR_DesIn(void)         {return &m_linkR_DesIn_share;}
    
    // Shared OutPort Data (Methods)

    SharedDataBase* linkState_DesOut(void)    {return &m_linkState_DesOut_share;}
    SharedDataBase* linkStateLin_DesOut(void) {return &m_linkStateLin_DesOut_share;}
    SharedDataBase* linkStateAng_DesOut(void) {return &m_linkStateAng_DesOut_share;}

    SharedDataBase* linkPose_DesOut(void)     {return &m_linkPose_DesOut_share;}
    SharedDataBase* linkP_DesOut(void)        {return &m_linkP_DesOut_share;}
    SharedDataBase* linkR_DesOut(void)        {return &m_linkR_DesOut_share;}

    SharedDataBase* linkState_HatOut(void)    {return &m_linkState_HatOut_share;}
    SharedDataBase* linkStateLin_HatOut(void) {return &m_linkStateLin_HatOut_share;}
    SharedDataBase* linkStateAng_HatOut(void) {return &m_linkStateAng_HatOut_share;}
    
    SharedDataBase* linkState_ErrOut(void)    {return &m_linkState_ErrOut_share;}
    SharedDataBase* linkStateLin_ErrOut(void) {return &m_linkStateLin_ErrOut_share;}
    SharedDataBase* linkStateAng_ErrOut(void) {return &m_linkStateAng_ErrOut_share;}

    SharedDataBase* linkState_RefOut(void)    {return &m_linkState_RefOut_share;}
    SharedDataBase* linkStateLin_RefOut(void) {return &m_linkStateLin_RefOut_share;}
    SharedDataBase* linkStateAng_RefOut(void) {return &m_linkStateAng_RefOut_share;}
    
    SharedDataBase* linkPose_RefOut(void)     {return &m_linkPose_RefOut_share;}
    SharedDataBase* linkP_RefOut(void)        {return &m_linkP_RefOut_share;}
    SharedDataBase* linkR_RefOut(void)        {return &m_linkR_RefOut_share;}
    
  protected:

    // Shared InPort Data (Variables)

    State* m_linkState_DesIn;
    SharedData<State*> m_linkState_DesIn_share;
    
    StatePVA* m_linkStateLin_DesIn;
    SharedData<StatePVA*> m_linkStateLin_DesIn_share;

    StateAttPVA* m_linkStateAng_DesIn;
    SharedData<StateAttPVA*> m_linkStateAng_DesIn_share;

    StatePosAtt* m_linkPose_DesIn;
    SharedData<StatePosAtt*> m_linkPose_DesIn_share;

    hrp::Vector3* m_linkP_DesIn;
    SharedData<hrp::Vector3*> m_linkP_DesIn_share;
    
    hrp::Matrix33* m_linkR_DesIn;
    SharedData<hrp::Matrix33*> m_linkR_DesIn_share;
    
    // Shared OutPort Data (Variables)

    State                     m_linkState_DesOut;
    SharedData<State>         m_linkState_DesOut_share;
    SharedData<StatePVA>      m_linkStateLin_DesOut_share;
    SharedData<StateAttPVA>   m_linkStateAng_DesOut_share;
    SharedData<StatePosAtt>   m_linkPose_DesOut_share;
    SharedData<hrp::Vector3>  m_linkP_DesOut_share;
    SharedData<hrp::Matrix33> m_linkR_DesOut_share;
    
    State                     m_linkState_HatOut;
    SharedData<State>         m_linkState_HatOut_share;
    SharedData<StatePVA>      m_linkStateLin_HatOut_share;
    SharedData<StateAttPVA>   m_linkStateAng_HatOut_share;

    State                     m_linkState_ErrOut;
    SharedData<State>         m_linkState_ErrOut_share;
    SharedData<StatePVA>      m_linkStateLin_ErrOut_share;
    SharedData<StateAttPVA>   m_linkStateAng_ErrOut_share;

    State                     m_linkState_RefOut;
    SharedData<State>         m_linkState_RefOut_share;
    SharedData<StatePVA>      m_linkStateLin_RefOut_share;
    SharedData<StateAttPVA>   m_linkStateAng_RefOut_share;
    SharedData<StatePosAtt>   m_linkPose_RefOut_share;
    SharedData<hrp::Vector3>  m_linkP_RefOut_share;
    SharedData<hrp::Matrix33> m_linkR_RefOut_share;
    
  private:

    mc_tasks::PositionTask* positionTask();
    mc_tasks::OrientationTask* orientationTask();

    // Method Functions

    bool cmd_set_position(std::istringstream& i_strm, std::ostringstream& o_strm);
    bool cmd_set_orientation(std::istringstream& i_strm, std::ostringstream& o_strm);

    bool cmd_shift_position(std::istringstream& i_strm, std::ostringstream& o_strm);
  };
  
}

#endif // __POSE_TASK_H__
