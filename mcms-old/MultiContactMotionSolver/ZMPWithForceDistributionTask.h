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

#ifndef __ZMP_WiTH_FORCE_DISTRIBUTION_TASK__
#define __ZMP_WITH_FORCE_DISTRIBUTION_TASK__

#include "ForceDistributionTaskBase.h"

namespace multi_contact_motion_solver {

  class ZMPWithForceDistributionTask : public ForceDistributionTaskBase {

  public:

    ZMPWithForceDistributionTask(common_interface::TaskExecutionManager* parentManager,
                                 const char* name = "zmp-with-force-distribution-task",
                                 std::list<std::string>* cmd_init = NULL);

    bool onExecute(void) override;

    // Shared InPort Data (Methods)

    SharedDataBase* zmp_DesIn(void)     {return &m_zmp_DesIn_share;}
    SharedDataBase* zmp_ModIn(void)     {return &m_zmp_ModIn_share;}

    // Shared OutPort Data (Methods)

    SharedDataBase* zmp_DesOut(void)    {return &m_zmp_DesOut_share;}
    SharedDataBase* zmp_ModOut(void)    {return &m_zmp_ModOut_share;}
    SharedDataBase* zmp_RefOut(void)    {return &m_zmp_RefOut_share;}

    SharedDataBase* zmp_CalOut(void)    {return &m_zmp_CalOut_share;}
    SharedDataBase* zmp_ErrOut(void)    {return &m_zmp_ErrOut_share;}
    SharedDataBase* moment_ErrOut(void) {return &m_moment_ErrOut_share;}

  private:

    // Shared InPort Data (Variables)

    hrp::Vector3* m_zmp_DesIn;
    SharedData<hrp::Vector3*> m_zmp_DesIn_share;
    hrp::Vector3* m_zmp_ModIn;
    SharedData<hrp::Vector3*> m_zmp_ModIn_share;

    // Shared OutPort Data (Variables)

    hrp::Vector3 m_zmp_DesOut;
    SharedData<hrp::Vector3> m_zmp_DesOut_share;
    hrp::Vector3 m_zmp_ModOut;
    SharedData<hrp::Vector3> m_zmp_ModOut_share;
    hrp::Vector3 m_zmp_RefOut;
    SharedData<hrp::Vector3> m_zmp_RefOut_share;

    hrp::Vector3 m_zmp_CalOut;
    SharedData<hrp::Vector3> m_zmp_CalOut_share;
    hrp::Vector3 m_zmp_ErrOut;
    SharedData<hrp::Vector3> m_zmp_ErrOut_share;
    hrp::Vector3 m_moment_ErrOut;
    SharedData<hrp::Vector3> m_moment_ErrOut_share;
    
    // Task related

    hrp::Vector3 m_zmp_default;

    // Method Functions

    /** Set the desired \f$ zmp \f$.
     *
     *  Usage:
     *  ":set-zmp zmp"
     */
    bool cmd_set_zmp(std::istringstream& i_strm, std::ostringstream& o_strm);
    
    /** Set the weight \f$ w_i \f$ for each dimension of the zmp task.
     *
     *  Usage:
     *  ":set-dim-weight w_0 w_1 w_2"
     */
    bool cmd_set_dim_weight(std::istringstream& i_strm, std::ostringstream& o_strm);
  };
  
}

#endif // __ZMP_WITH_FORCE_DISTRIBUTION_TASK__
