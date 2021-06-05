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

#ifndef __PRIMITIVE_END_EFFECTOR_POSE_H__
#define __PRIMITIVE_END_EFFECTOR_POSE_H__

#include "../MultiContactStatePrimitiveBase.h"

namespace trajectory_generator {
  class PositionTrajectoryGenerator;
  class OrientationTrajectoryGenerator;
}

namespace multi_contact_state_manager {

  class PrimitiveEndEffectorPose : public MultiContactStatePrimitiveBase {

  public:

    PrimitiveEndEffectorPose(MultiContactStateManager* mcsManager,
			     MultiContactStatePrimitiveGroup* mcsGroup,
			     MultiContactStatePrimitiveLoaderBase* loader,
			     MultiContactStateTransition* parentState,
			     const std::string& name, const std::string& guid);
    ~PrimitiveEndEffectorPose(void) {}

    void registerPositionInterpolator(trajectory_generator::PositionTrajectoryGenerator* interpolator)
    { m_position_interpolator = interpolator; }

    void setPositionInterpolationTiming(double t0, double tf)
    {
      m_posInterpolationTiming.t0 = t0;
      m_posInterpolationTiming.tf = tf;
    }
    
    void addDesPosition(hrp::Vector3 pos)
    { m_desPositions.push_back(pos); }

    void setDesFinalLinearVelocity(hrp::Vector3 linVel)
    { m_desFinalLinearVelocity = linVel; }

    void registerOrientationInterpolator(trajectory_generator::OrientationTrajectoryGenerator* interpolator)
    { m_orientation_interpolator = interpolator; }
    
    void setOrientationInterpolationTiming(double t0, double tf)
    {
      m_rpyInterpolationTiming.t0 = t0;
      m_rpyInterpolationTiming.tf = tf;
    }
    
    void addDesOrientation(hrp::Vector3 rpy)
    { m_desOrientations.push_back(rpy); }

    void setDesFinalAngularVelocity(hrp::Vector3 angVel)
    { m_desFinalAngularVelocity = angVel; }

  private:

    struct Interval {
      double t0;
      double tf;
    };
    
    trajectory_generator::PositionTrajectoryGenerator* m_position_interpolator;
    trajectory_generator::OrientationTrajectoryGenerator* m_orientation_interpolator;

    Interval m_posInterpolationTiming;
    std::vector<hrp::Vector3> m_desPositions;
    hrp::Vector3 m_desFinalLinearVelocity;

    Interval m_rpyInterpolationTiming;
    std::vector<hrp::Vector3> m_desOrientations;
    hrp::Vector3 m_desFinalAngularVelocity;
    
    // Primitive Behavior Related
    
    void onEntry(void);
    void onExit(void);
    bool check(int n_time, const std::bitset<NUM_MULTI_CONTACT_STATE_STOP_ACTIONS>& signal_state);
  };
  
}

#endif // __PRIMITIVE_END_EFFECTOR_POSE_H__
