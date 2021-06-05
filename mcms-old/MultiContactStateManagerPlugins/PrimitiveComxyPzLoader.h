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

#ifndef __PRIMITIVE_COMXY_PZ_LOADER_H__
#define __PRIMITIVE_COMXY_PZ_LOADER_H__

#include "../MultiContactStatePrimitiveLoaderBase.h"

namespace cnoid {
  class YamlMapping;
}

namespace multi_contact_state_manager {
  
  class PrimitiveComxyPz;

  class PrimitiveComxyPzLoader : public MultiContactStatePrimitiveLoaderBase {

  public:

    PrimitiveComxyPzLoader(common_interface::TaskExecutionManager* parentManager,
			   const char* name = "mcs-comxy-pz-mcms",
			   std::list<std::string>* cmd_init = NULL);
    ~PrimitiveComxyPzLoader(void) {}

    MultiContactStatePrimitiveBase* loadPrimitive(const std::string& stateId, const std::string& guid,
						  MultiContactStatePrimitiveGroup* mcsGroup,
						  MultiContactStateTransition* parentState,
						  const cnoid::YamlMapping& archive,
						  std::ostringstream& o_strm);
  };
  
}

#endif // __PRIMITIVE_COMXY_PZ_LOADER_H__
