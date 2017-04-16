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

#include "ContactManager.h"

void ContactManager::addContact(Contact* contact)
{
  std::vector<Contact*>::iterator it;
  for (it = m_contact.begin(); it != m_contact.end(); it++) {
    if (*it == contact)
      return;
  }
  m_contact.push_back(contact);
}

bool ContactManager::removeContact(Contact* contact)
{
  std::vector<Contact*>::iterator it;
  for (it = m_contact.begin(); it != m_contact.end(); it++) {
    if(*it == contact) {
      m_contact.erase(it);
      return true;
    }
  }
  return false;
}

int ContactManager::numActiveContacts()
{
  int num;
  std::vector<Contact*>::iterator it;
  //for (it = m_contact.begin(); it != m_contact.end(); it++)
    //if(it->isActive())
    //num++;
  return num;
}

hrp::dmatrix& wrenchMatrix()
{
  std::vector<Contact*>::iterator it;
  //for (it = m_contact.begin(); it != m_contact.end(); it++)
    //if(it->isActive()) {
    //Incomplete
    //}
}
