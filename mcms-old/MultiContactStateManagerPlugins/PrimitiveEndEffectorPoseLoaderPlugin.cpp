// -*- mode: c++; indent-tabs-mode: nil; tab-width: 2; c-basic-offset: 2; -*-
/*
 * Copyright (c) 2019,
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

#include <MotionGenerator/MultiContactStateManager/MultiContactStatePrimitives/MCMS/PrimitiveEndEffectorPoseLoader.h>

using namespace common_interface;
using namespace multi_contact_state_manager;

TaskExecutionHandler* create_task(TaskExecutionManager* parentManager,
				  const char* name = "mcs-endeffector-pose-mcms",
				  std::list<std::string>* cmd_init = NULL)
{
  return new PrimitiveEndEffectorPoseLoader(parentManager, name, cmd_init);
}
