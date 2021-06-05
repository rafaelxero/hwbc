/*
 * Copyright (c) 2013,
 * @author Mitsuharu Morisawa
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
#include "BalanceControllerCapturePoint.h"
#include <HumanoidMotionControlCore/HumanoidMotionControlCore.h>
#include <Util/MiscString.h>
#include <Math/MathFunction.h>
#include <Math/Physics.h>

using namespace hrp;
using namespace humanoid_motion_control_core;
using namespace common_interface;
using namespace motion_control;

BalanceControllerCapturePoint::
BalanceControllerCapturePoint(TaskExecutionManager* parentManager,
                              const char *name,
                              std::list<std::string>* cmd_init) :
  BalanceControllerPoleAssignBase(3, parentManager, name, cmd_init),
  m_compensateCapturePointError(0.0),
  m_LPFgain_omega(lround(0.7/m_dt)),
  m_limit_ratio_gravity(0.0),
  m_zmp_uncomp_threshold(5.0e-3),
  m_omega_zmp(20.0),
  m_omega_cpsum(0.0),
  m_omega(3.5),  // nan is assigned in case of sqrt(m_omega2) in Ubuntu10.04
  m_omega2(Gv/0.8),
  m_omega_share(m_omega),
  m_CapturePoint_sum(Vector3::Zero()),
  m_CapturePoint_sum_share(m_CapturePoint_sum),
  m_Zmp_balance(Vector3::Zero()),
  m_Zmp_balance_share(m_Zmp_balance),
  m_Zmp_offset_cp(Vector3::Zero()),
  m_Zmp_offset_cp_share(m_Zmp_offset_cp),
  m_Zmp_out(Vector3::Zero()),
  m_Zmp_out_share(m_Zmp_out),
  m_Torque_ctrl_ref(NULL),
  m_Torque_ctrl_ref_share(m_Torque_ctrl_ref),
  m_Torque_uncomp(NULL),
  m_Torque_uncomp_share(m_Torque_uncomp),
  m_COG_omega(NULL),
  m_COG_omega_share(m_COG_omega),
  m_VCOGdot_omega(NULL),
  m_VCOGdot_omega_share(m_VCOGdot_omega),
  m_CapturePoint_ref(Vector3::Zero()),
  m_CapturePoint_res(Vector3::Zero()),
  m_VCapturePoint_ref(Vector3::Zero()),
  m_VCapturePoint_res(Vector3::Zero()),
  m_VCapturePoint_err(Vector3::Zero())
{
  registerMethodFunction(":omega-zmp",
                         (methodFuncPtr)&BalanceControllerCapturePoint::
                         cmd_set_omega_zmp);
  registerMethodFunction(":omega-cpsum",
                         (methodFuncPtr)&BalanceControllerCapturePoint::
                         cmd_set_omega_cpsum);
  registerMethodFunction(":omega-lpf",
                         (methodFuncPtr)&BalanceControllerCapturePoint::
                         cmd_set_omega_lpf_gain);
  registerMethodFunction(":omega-lpf-ratio",
                         (methodFuncPtr)&BalanceControllerCapturePoint::
                         cmd_set_omega_lpf_gain_ratio);
  registerMethodFunction(":limit-ratio-gravity",
                         (methodFuncPtr)&BalanceControllerCapturePoint::
                         cmd_limit_ratio_gravity);
  registerMethodFunction(":set-zmp-uncomp-threshold",
                         (methodFuncPtr)&BalanceControllerCapturePoint::
                         cmd_set_zmp_uncomp_threshold);
  registerMethodFunction(":compensate-capture-point-error",
                         (methodFuncPtr)&BalanceControllerCapturePoint::
                         cmd_compensate_capture_point_error);
  
  registerInPort("Torque_ctrl_ref",
                 (sharedFuncPtr)&BalanceControllerCapturePoint::TorqueControlRefIn);
  registerInPort("Torque_uncomp",
                 (sharedFuncPtr)&BalanceControllerCapturePoint::TorqueUncompensationIn);
  registerInPort("COG_omega",
                 (sharedFuncPtr)&BalanceControllerCapturePoint::COGOmegaIn);
  registerInPort("VCOGdot_omega",
                 (sharedFuncPtr)&BalanceControllerCapturePoint::VCOGdotOmegaIn);
  
  registerOutPort("omega",
                  (sharedFuncPtr)&BalanceControllerCapturePoint::OmegaOut);
  registerOutPort("CapturePoint_sum",
                  (sharedFuncPtr)&BalanceControllerCapturePoint::CapturePointSumOut);
  registerOutPort("Zmp_balance",
                  (sharedFuncPtr)&BalanceControllerCapturePoint::ZmpBalanceOut);
  registerOutPort("Zmp_offset_cp",
                  (sharedFuncPtr)&BalanceControllerCapturePoint::ZmpOffsetCapturePointOut);
  registerOutPort("Zmp_out",
                  (sharedFuncPtr)&BalanceControllerCapturePoint::ZmpOutputOut);
  
  initialCommands(cmd_init);
}

bool BalanceControllerCapturePoint::
onActivate(void)
{
  if( !hmc->checkReservedInPort(TorqueControlRefIn()) ){
    std::cout << "\e[1;31m!!!ERROR!!!\e[m " << TorqueControlRefIn()->getPortName()
              << " is not connected in " << getTaskName() << std::endl;
    return false;
  }
  if( !hmc->checkReservedInPort(TorqueUncompensationIn()) ){
    std::cout << "\e[1;31m!!!ERROR!!!\e[m " << TorqueUncompensationIn()->getPortName()
              << " is not connected in " << getTaskName() << std::endl;
    return false;
  }
  
  if( !hmc->checkReservedInPort(COGOmegaIn()) ){
    std::cout << "\e[1;31m!!!ERROR!!!\e[m " << COGOmegaIn()->getPortName()
              << " is not connected in " << getTaskName() << std::endl;
    return false;
  }
  if( !hmc->checkReservedInPort(VCOGdotOmegaIn()) ){
    std::cout << "\e[1;31m!!!ERROR!!!\e[m " << VCOGdotOmegaIn()->getPortName()
              << " is not connected in " << getTaskName() << std::endl;
    return false;
  }
  
  if( !hmc->checkReservedInPort(COGAccelReferenceIn()) ){
    std::cout << "\e[1;31m!!!ERROR!!!\e[m " << COGAccelReferenceIn()->getPortName()
              << " is not connected in " << getTaskName() << std::endl;
    return false;
  }
  if( !hmc->checkReservedInPort(COGAccelResponseIn()) ){
    std::cout << "\e[1;31m!!!ERROR!!!\e[m " << COGAccelResponseIn()->getPortName()
              << " is not connected in " << getTaskName() << std::endl;
    return false;
  }
  
  return BalanceControllerPoleAssignBase::onActivate();
}

bool BalanceControllerCapturePoint::
isReadyToActivate(void)
{
  if( control_execution ){
    BalanceControllerCapturePoint::onExecute();
    return true;
  }
  else{
    m_Zmp_balance.setZero();
    m_Zmp_out.setZero();
    m_CapturePoint_err.setZero();
    m_VCapturePoint_err.setZero();
    m_CapturePoint_sum.setZero();
    m_Zmp_offset_cp.setZero();
    
    return BalanceControllerPoleAssignBase::isReadyToActivate();
  }
}

bool BalanceControllerCapturePoint::
isReadyToDeactivate(void)
{
  return BalanceControllerBase::isReadyToDeactivate();
}

bool BalanceControllerCapturePoint::
onExecute(void)
{
  setupGain();
  
  if( (control_request || control_execution) && (*m_onGroundFeet) ){
    double VCOGdot_z = (*m_VCOGdot_omega)(Z);
    if( VCOGdot_z > Gv * m_limit_ratio_gravity )
      VCOGdot_z =  Gv * m_limit_ratio_gravity;
    else if( VCOGdot_z < -Gv * m_limit_ratio_gravity )
      VCOGdot_z = -Gv * m_limit_ratio_gravity;
    
    const double omega2 = (Gv + VCOGdot_z) / ((*m_COG_omega)(Z) - (*m_Zmp_ref)(Z));
    LowPassFilter(omega2, m_omega2, m_LPFgain_omega*m_dt);
    m_omega  = sqrt(m_omega2);
    
    Vector3 Zmp_uncomp(Vector3::Zero());
    Zmp_uncomp <<
      -(*m_Torque_uncomp)(Y) / m_Force_gravity(Z),
       (*m_Torque_uncomp)(X) / m_Force_gravity(Z),
       0.0;

    /*
    bool nantest = (isnan(Zmp_uncomp.array())).count() > 0;  // Rafa added this
    
    if (nantest) {
      std::cout << "Rafa, in  BalanceControllerCapturePoint::onExecute, Zmp_uncomp = "
		<< Zmp_uncomp.transpose() << ", *m_Torque_uncomp = " << m_Torque_uncomp->transpose()
		<< ", Force_gravity(Z) = " << Force_gravity(Z) << std::endl;
      *(int*)0 = 0;  // Rafa added this
    }
    */
        
    for( int axis = X ; axis <= Y ; axis++ ){
      m_COG_err.P(axis)    = (*m_COG_ref)(axis) - (*m_COG_res)(axis);
      m_COG_err.V(axis)    = (*m_VCOG_ref)(axis) - (*m_VCOG_res)(axis);
      m_COG_err.Vdot(axis) = (*m_VCOGdot_ref)(axis) - (*m_VCOGdot_res)(axis);
      m_Zmp_err(axis)      = (*m_Zmp_ref)(axis) - (*m_Zmp_res)(axis);
      
      //double CapturePoint_res_pre(m_CapturePoint_res(axis));
      m_CapturePoint_ref(axis)  = (*m_COG_ref)(axis)  + (*m_VCOG_ref)(axis) / m_omega;
      m_VCapturePoint_ref(axis) = (*m_VCOG_ref)(axis) + (*m_VCOGdot_ref)(axis) / m_omega;
      
      m_CapturePoint_res(axis)  = (*m_COG_res)(axis)  + (*m_VCOG_res)(axis) / m_omega;
      m_VCapturePoint_res(axis) = (*m_VCOG_res)(axis) + (*m_VCOGdot_res)(axis) / m_omega;
      
      //std::cout << "Capture Point res(" << axis << ") = " << m_CapturePoint_res(axis) << std::endl;
      //std::cout << "VCapture Point res(" << axis << ") = " << m_VCapturePoint_res(axis) << std::endl;
      
      m_CapturePoint_err(axis)  = m_CapturePoint_ref(axis)  - m_CapturePoint_res(axis);
      m_VCapturePoint_err(axis) = m_VCapturePoint_ref(axis) - m_VCapturePoint_res(axis);
      
      const double& alpha = pole[axis](0);
      const double& beta  = pole[axis](1);
      const double& gamma = pole[axis](2);
      Vector3 K;
      K <<
        -alpha*beta*gamma/(m_omega*m_omega_zmp),
        -(alpha*beta + beta*gamma + gamma*alpha + m_omega*m_omega_zmp) / (m_omega*m_omega_zmp),
        -(alpha + beta + gamma + m_omega - m_omega_zmp) / (m_omega*m_omega_zmp);

      //std::cout << "Rafa, in BalanceControllerCapturePoint::onExecute, m_CapturePoint_sum = "
      //          << m_CapturePoint_sum.transpose() << std::endl;
      
      Vector3 x;
      x <<
        m_CapturePoint_sum(axis),
        m_CapturePoint_err(axis),
        m_VCapturePoint_err(axis);
      m_Zmp_balance(axis) = K.dot(x);

      /*
      bool nantest = (isnan(m_Zmp_balance.array())).count() > 0;  // Rafa added this

      if (nantest)
	*(int*)0 = 0;  // Rafa added this
      */
      
      if( fabs(Zmp_uncomp(axis)) < m_zmp_uncomp_threshold )
        m_CapturePoint_sum(axis) += m_compensateCapturePointError * x(1) * m_dt;
      else {
        LowPassFilter(Zmp_uncomp(axis)/K(0), m_CapturePoint_sum(axis), m_omega_cpsum * m_dt);
	/*
	std::cout << "Rafa, in BalanceControllerCapturePoint::onExecute, Zmp_uncomp(axis) >= th, "
		  << "Zmp_uncomp(" << axis << ") = " << Zmp_uncomp(axis) << ", "
		  << "K(0) = " << K(0) << ", m_CapturePoint_sum(axis) = "
		  <<  m_CapturePoint_sum(axis) << ", m_omega_cpsum = " << m_omega_cpsum
		  << "m_dt = " << m_dt << std::endl;
	*/
      }
      m_Zmp_offset_cp(axis) = -K(0) * m_CapturePoint_sum(axis);
    }
    
#if 0
    Vector3 DivCheck(m_Torque_uncomp->cross(m_CapturePoint_err));
    if( DivCheck(axis) >= 0.0 )
      m_CapturePoint_sum(axis) += m_compensateCapturePointError * m_CapturePoint_err(axis) * m_dt;
#endif    
    
#if 0
    //std::cout << "CapturePoint err = " << m_CapturePoint_err.transpose() << std::endl;
    //std::cout << "VCapturePoint err = " << m_VCapturePoint_err.transpose() << std::endl;
    
    std::cout << "balance::COG_ref = " << m_COG_ref->transpose() << std::endl;
    std::cout << "balance::COG_res = " << m_COG_res->transpose() << std::endl;
    //std::cout << "CapturePoint_ref = " << m_CapturePoint_ref.transpose() << std::endl;
    //std::cout << "CapturePoint_res = " << m_CapturePoint_res.transpose() << std::endl;
    
    std::cout << "Zmp_balance = " << m_Zmp_balance.transpose() << std::endl;
#endif
    
    //std::cout << "Zmp_balance = " << m_Zmp_balance(X) << "," << m_Zmp_balance(Y) << std::endl;
  }
  else{
    m_Zmp_balance.setZero();
    m_CapturePoint_err.setZero();
    m_VCapturePoint_err.setZero();
    m_CapturePoint_sum *= 0.99;
    m_Zmp_offset_cp.setZero();
  }
  
  m_Zmp_out = *m_Zmp_ref + m_Zmp_balance;
  m_Torque_ref = m_Zmp_balance.cross(m_Force_gravity);
  *m_Torque_ctrl_ref += m_Torque_ref;
  
  return BalanceControllerPoleAssignBase::onExecute();
}

void BalanceControllerCapturePoint::
dumpCommandParameters(std::ostringstream& o_strm)
{
  BalanceControllerPoleAssignBase::dumpCommandParameters(o_strm);
  
  o_strm << ":omega-zmp : " << m_omega_zmp << std::endl;
  o_strm << ":omega-lpf-ratio : " << m_LPFgain_omega * m_dt << std::endl;
  o_strm << ":limit-ratio-gravity : " << m_limit_ratio_gravity << std::endl;
  //o_strm << ":compensate-capture-point-error " << m_compensateCapturePointError << std::endl;
}

bool BalanceControllerCapturePoint::
cmd_set_omega_zmp(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  i_strm >> m_omega_zmp;
  o_strm << "set omega-zmp : " << m_omega_zmp << std::endl;
  return true;
}

bool BalanceControllerCapturePoint::
cmd_set_omega_cpsum(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  i_strm >> m_omega_cpsum;
  o_strm << "set omega-cpsum : " << m_omega_cpsum << std::endl;
  return true;
}

bool BalanceControllerCapturePoint::
cmd_set_omega_lpf_gain(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  i_strm >> m_LPFgain_omega;
  o_strm << "set omega-lpf : " << m_LPFgain_omega << std::endl;
  return true;
}

bool BalanceControllerCapturePoint::
cmd_set_omega_lpf_gain_ratio(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  double ratio = 0.0;
  i_strm >> ratio;
  m_LPFgain_omega = ratio / m_dt;
  o_strm << "set omega-lpf : " << m_LPFgain_omega << std::endl;
  return true;
}

bool BalanceControllerCapturePoint::
cmd_limit_ratio_gravity(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  i_strm >> m_limit_ratio_gravity;
  o_strm << "set limit-ratio-gravity : " << m_limit_ratio_gravity << std::endl;
  return true;
}

bool BalanceControllerCapturePoint::
cmd_set_zmp_uncomp_threshold(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  i_strm >> m_zmp_uncomp_threshold;
  o_strm << "set zmp-uncomp-threshold : " << m_zmp_uncomp_threshold << std::endl;
  return true;
}

bool BalanceControllerCapturePoint::
cmd_compensate_capture_point_error(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  std::string flg;
  i_strm >> flg;
  if( isOn(flg) ){
    m_compensateCapturePointError = 1.0;
    o_strm << "compensate CapturePoint error" << std::endl;
  }
  else{
    m_compensateCapturePointError = 0.0;
    o_strm << "don't compensate CapturePoint error" << std::endl;
  }
  return true;
}
