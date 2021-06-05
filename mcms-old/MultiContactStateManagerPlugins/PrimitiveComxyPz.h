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

#ifndef __PRIMITIVE_COMXY_PZ_H__
#define __PRIMITIVE_COMXY_PZ_H__

#include "../MultiContactStatePrimitiveBase.h"

namespace trajectory_generator {
  class PositionTrajectoryGenerator;
}

namespace multi_contact_state_manager {

  class PrimitiveComxyPz : public MultiContactStatePrimitiveBase {

  public:

    PrimitiveComxyPz(MultiContactStateManager* mcsManager,
		     MultiContactStatePrimitiveGroup* mcsGroup,
		     MultiContactStatePrimitiveLoaderBase* loader,
		     MultiContactStateTransition* parentState,
		     const std::string& name, const std::string& guid);
    ~PrimitiveComxyPz(void) {}

    void registerComInterpolator(trajectory_generator::PositionTrajectoryGenerator* interpolator)
    { m_com_interpolator = interpolator; }
    
    void setComInterpolationTiming(double t0, double tf)
    {
      m_com_posInterpolationTiming.t0 = t0;
      m_com_posInterpolationTiming.tf = tf;
    }
    
    void addComDesPosition(hrp::Vector3 pos)
    { m_com_desPositions.push_back(pos); }

    void setComDesFinalLinearVelocity(hrp::Vector3 linVel)
    { m_com_desFinalLinearVelocity = linVel; }

    void registerBodyPositionInterpolator(trajectory_generator::PositionTrajectoryGenerator* interpolator)
    { m_bodyPosition_interpolator = interpolator; }
    
    void setBodyPositionInterpolationTiming(double t0, double tf)
    {
      m_body_posInterpolationTiming.t0 = t0;
      m_body_posInterpolationTiming.tf = tf;
    }
    
    void addBodyDesPosition(hrp::Vector3 pos)
    { m_body_desPositions.push_back(pos); }

    void setBodyDesFinalLinearVelocity(hrp::Vector3 linVel)
    { m_body_desFinalLinearVelocity = linVel; }

    void registerZmpInterpolator(trajectory_generator::PositionTrajectoryGenerator* interpolator)
    { m_zmp_interpolator = interpolator; }
    
    void setZmpInterpolationTiming(double t0, double tf)
    {
      m_zmp_posInterpolationTiming.t0 = t0;
      m_zmp_posInterpolationTiming.tf = tf;
    }
    
    void addZmpDesPosition(hrp::Vector3 pos)
    { m_zmp_desPositions.push_back(pos); }

    void setZmpDesFinalLinearVelocity(hrp::Vector3 linVel)
    { m_zmp_desFinalLinearVelocity = linVel; }    

  private:

    struct Interval {
      double t0;
      double tf;
    };
    
    trajectory_generator::PositionTrajectoryGenerator* m_com_interpolator;
    trajectory_generator::PositionTrajectoryGenerator* m_bodyPosition_interpolator;
    trajectory_generator::PositionTrajectoryGenerator* m_zmp_interpolator;

    Interval m_com_posInterpolationTiming;
    std::vector<hrp::Vector3> m_com_desPositions;
    hrp::Vector3 m_com_desFinalLinearVelocity;

    Interval m_body_posInterpolationTiming;
    std::vector<hrp::Vector3> m_body_desPositions;
    hrp::Vector3 m_body_desFinalLinearVelocity;

    Interval m_zmp_posInterpolationTiming;
    std::vector<hrp::Vector3> m_zmp_desPositions;
    hrp::Vector3 m_zmp_desFinalLinearVelocity;
    
    // Primitive Behavior Related
    
    void onEntry(void);
    void onExit(void);
    bool check(int n_time, const std::bitset<NUM_MULTI_CONTACT_STATE_STOP_ACTIONS>& signal_state);
  };
  
}

#endif // __PRIMITIVE_COMXY_PZ_H__
