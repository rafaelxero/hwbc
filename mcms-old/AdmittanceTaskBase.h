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

#ifndef __ADMITTANCE_TASK_BASE_H__
#define __ADMITTANCE_TASK_BASE_H__

#include "WrenchSpaceTask.h"
#include <Math/Filter/LPFilter.h>

namespace multi_contact_motion_solver {

  class AdmittanceTaskBase : public WrenchSpaceTask {

  public:

    AdmittanceTaskBase(common_interface::TaskExecutionManager* parentManager,
                       const char* name = "admittance-task-base",
                       std::list<std::string>* cmd_init = NULL);
    
    virtual bool onExecute(void) = 0;
    
    // Shared OutPort Data (Methods)
    
    SharedDataBase* force_CalOut(void) {return &m_force_CalOut_share;}
    SharedDataBase* moment_CalOut(void) {return &m_moment_CalOut_share;}

    // Task Related

    const sva::ForceVecd & measuredWrench()
    {
      m_estimated_beforehand = true;
      estimateMeasuredWrench();
      return m_measuredWrench;
    }
    
  protected:

    // Shared OutPort Data (Variables)

    hrp::Vector3 m_force_CalOut;
    SharedData<hrp::Vector3> m_force_CalOut_share;

    hrp::Vector3 m_moment_CalOut;
    SharedData<hrp::Vector3> m_moment_CalOut_share;

    // Task Related

    std::string m_sensorLink;
    
    double m_force_stiffness_default, m_force_damping_default;
    double m_moment_stiffness_default, m_moment_damping_default;

    sva::ForceVecd m_measuredWrench;
    
    filter::LPFilter m_force_LPFilter;
    filter::LPFilter m_moment_LPFilter;

    bool m_estimated_beforehand;
    
    void estimateMeasuredWrench();
    
    // Method Functions

    bool cmd_target_link(std::istringstream& i_strm, std::ostringstream& o_strm) override;
    
    /** Set the specified stiffness \f$ k_p \f$ and damping \f$ k_d \f$ for the force component of the Admittance Task.
     *
     *  Usage:
     *  ":set-force-pd-gains k_p k_d"
     */
    bool cmd_set_force_pd_gains(std::istringstream& i_strm, std::ostringstream& o_strm);

    /** Set the specified stiffness \f$ k_p \f$ and damping \f$ k_d \f$ for the moment component of the Admittance Task.
     *
     *  Usage:
     *  ":set-moment-pd-gains k_p k_d"
     */
    bool cmd_set_moment_pd_gains(std::istringstream& i_strm, std::ostringstream& o_strm);
    
    /** Set the weight \f$ w_i \f$ for each dimension of the current Task.
     *  Each weight is multiplied by the overall weight of the Task to affect each dimension.
     *
     *  Usage:
     *  ":set-dim-weight w_1 ... w_6"
     */
    bool cmd_set_dim_weight(std::istringstream& i_strm, std::ostringstream& o_strm);

    /** Specify the behavior of treating the desired settings as local, or not,
     *  by using a flag: \f$ local \f$.
     *
     *  Usage:
     *  ":treat-dimweight-as-local local"
     */
    bool cmd_treat_settings_as_local(std::istringstream& i_strm, std::ostringstream& o_strm);
  };

}

#endif // __ADMITTANCE_TASK_BASE_H__
