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

#ifndef __CONTACT_MANAGER__
#define __CONTACT_MANAGER__

#include "Contact.h"

class ContactManager {

 private:
    
  /// Array of the possible contacts (active and not active)
  std::vector<Contact*> m_contact;
  
  /// Wrench matrices for the active contacts arranged into a single matrix
  hrp::dmatrix m_wrenchMatrix;
    
 public:
    
  ContactManager() {};
  ~ContactManager() {};
  
  /// Add a possible contact to the set if not already considered
  void addContact(Contact* contact);

  /// Removes a possible contact from the set
  bool removeContact(Contact* contact);
  
  /// Returns the number of possible contacts (active or inactive)
  int numContacts() {return m_contact.size();}
  
  /// Returns the number of active contacts
  int numActiveContacts();

  /// Constructs (and returns) the matrix comprising the wrench matrices of the active contacts
  hrp::dmatrix& wrenchMatrix();
};

#endif  // __CONTACT_MANAGER__
