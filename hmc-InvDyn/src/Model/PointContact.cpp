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

#include <iostream>
#include "PointContact.h"

PointContact::PointContact(hrp::Link* targetLink, hrp::Vector3 localPosition, int basisDim, double frictionCoeff) :
  Contact(targetLink), m_localPosition(localPosition)
{
  makeFrictionPyramidBasis(basisDim, frictionCoeff);
}

void PointContact::makeFrictionPyramidBasis(int basisDim, double frictionCoeff)
{
  for (int i = 0; i < basisDim; i++) {
    double angle = 2.0 * M_PI * (double) i / (double) basisDim;
    double mux = frictionCoeff * cos(angle);
    double muy = frictionCoeff * sin(angle);
    double den = sqrt(1.0 + frictionCoeff * frictionCoeff);
    m_frictionPyramidBasis.push_back(hrp::Vector3(mux/den, muy/den, 1.0/den));
  }
}

hrp::dmatrix& PointContact::wrenchMatrix()
{
  m_wrenchMatrix.resize(6, m_frictionPyramidBasis.size());
  
  for (int i = 0; i < m_frictionPyramidBasis.size(); i++) {
    m_wrenchMatrix.block<3, 1>(0, i) = m_targetLink->R * m_frictionPyramidBasis[i];
    m_wrenchMatrix.block<3, 1>(3, i) = m_targetLink->R * m_localPosition.cross(m_frictionPyramidBasis[i]) + m_targetLink->p.cross(m_wrenchMatrix.block<3, 1>(0, i));
  }

  return m_wrenchMatrix;
}
