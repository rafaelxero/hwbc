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

#ifndef __PRIMITIVE_CONTACT_MANAGER_LOADER_H__
#define __PRIMITIVE_CONTACT_MANAGER_LOADER_H__

#include "../MultiContactStatePrimitiveLoaderBase.h"

namespace cnoid {
  class YamlMapping;
}

namespace multi_contact_state_manager {
  
  class PrimitiveContactManager;

  class PrimitiveContactManagerLoader : public MultiContactStatePrimitiveLoaderBase {

  public:

    PrimitiveContactManagerLoader(common_interface::TaskExecutionManager* parentManager,
				   const char* name = "mcs-endeffector-pose-mcms",
				   std::list<std::string>* cmd_init = NULL);
    ~PrimitiveContactManagerLoader(void) {}

    MultiContactStatePrimitiveBase* loadPrimitive(const std::string& stateId, const std::string& guid,
						  MultiContactStatePrimitiveGroup* mcsGroup,
						  MultiContactStateTransition* parentState,
						  const cnoid::YamlMapping& archive,
						  std::ostringstream& o_strm);
  };
  
}

#endif // __PRIMITIVE_CONTACT_MANAGER_LOADER_H__
