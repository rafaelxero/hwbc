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

#ifndef __ADMITTANCE_TASK_H__
#define __ADMITTANCE_TASK_H__

#include "AdmittanceTaskBase.h"

namespace multi_contact_motion_solver {

  class AdmittanceTask : public AdmittanceTaskBase {

  public:

    AdmittanceTask(common_interface::TaskExecutionManager* parentManager,
                   const char* name = "admittance-task",
                   std::list<std::string>* cmd_init = NULL);

    bool onExecute(void);
  };

}

#endif // __ADMITTANCE_TASK_H__
