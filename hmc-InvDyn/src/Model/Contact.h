// -*- mode: c++; indent-tabs-mode: nil; tab-width: 2; c-basic-offset: 2; -*-
/*
 * Copyright (c) 2016,
 * @author Rafael Cisneros, Mitsuharu Morisawa
 *
 * AIST
 *
 * All rights reserved.
 *
 * This program is made available under the terms of the Eclipse Public License
 * v1.0 which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 */

#ifndef __CONTACT__
#define __CONTACT__

#include <hrpModel/Link.h>

class Contact {

protected:

  /// Specifies the link of the robot that is making contact (and determines the local reference frame of the contact)
  hrp::Link* m_targetLink;

  /// Contact state (true: making contact / false: not making contact)
  bool m_state;

public:
  
  Contact(void) : m_targetLink(NULL), m_state(false) {};
  Contact(hrp::Link* targetLink) : m_targetLink(targetLink), m_state(false) {};
  virtual ~Contact() {};

  /// Specifies the target link
  void setTargetLink(hrp::Link* targetLink) { m_targetLink = targetLink; }

  /// Changes the contact state to active (true) / inactive (false)
  void setContactState(bool state) {m_state = state;}

  /// Gets the actual contact state
  bool isActive() {return m_state;}
};

#endif  // __CONTACT__
