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

#ifndef __POINT_CONTACT__
#define __POINT_CONTACT__

#include "Contact.h"

class PointContact : public Contact {
  
private:
  
  /// Position of the contact in the target link's frame
  hrp::Vector3 m_localPosition;
  
  /// Basis defining the friction pyramid of this contact
  std::vector<hrp::Vector3> m_frictionPyramidBasis;

  /// Wrenches corresponding to the basis arranged as a matrix
  hrp::dmatrix m_wrenchMatrix;
  
  /// Force vector defined with respect to the friction pyramid basis
  std::vector<double> m_pyramidalForce;

  /// Constructs the set of unitary vectors for the basis defining the friction pyramid
  void makeFrictionPyramidBasis(int basisDim, double frictionCoeff);
  
public:
  
  PointContact(hrp::Link* targetLink, hrp::Vector3 localPosition, int basisDim, double frictionCoeff);
  ~PointContact() {};
  
  /// Constructs (and returns) the matrix comprising the wrenches corresponding to the basis
  hrp::dmatrix& wrenchMatrix();
};

#endif  // __POINT_CONTACT__
