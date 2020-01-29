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

#ifndef __NULL_SPACE_ADMITTANCE_TASK_H__
#define __NULL_SPACE_ADMITTANCE_TASK_H__

#include "AdmittanceTaskBase.h"
#include "AdmittanceManager.h"

namespace multi_contact_motion_solver {

  class NullSpaceAdmittanceTask : public AdmittanceTaskBase {

  public:

    NullSpaceAdmittanceTask(common_interface::TaskExecutionManager* parentManager,
			    const char* name = "null-spaceadmittance-task",
			    std::list<std::string>* cmd_init = NULL);

    bool onActivate(void) override;
    bool onExecute(void);

    // Shared InPort Data (Methods)

    SharedDataBase* fdistRatio_DesIn(void) {return &m_fdistRatio_DesIn_share;}

  private:

    AdmittanceManager* m_admittance_manager;
    
    // Shared InPort Data (Variables)

    hrp::Vector3* m_fdistRatio_DesIn;
    SharedData<hrp::Vector3*> m_fdistRatio_DesIn_share;
    
    // Method Functions
    
    /** Set the force distribution ratio (ratio_x, ratio_y, ratio_z) for the contact link.
     *
     *  Usage:
     *  ":set-fdist-ratio ratio_x ratio_y ratio_z"
     */
    bool cmd_set_fdist_ratio(std::istringstream& i_strm, std::ostringstream& o_strm);
  };

}

#endif // __NULL_SPACE_ADMITTANCE_TASK_H__
