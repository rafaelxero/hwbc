// -*- mode: c++; indent-tabs-mode: nil; tab-width: 2; c-basic-offset: 2; -*-
/*
 * Copyright (c) 2018
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

#ifndef __MULTI_CONTACT_MOTION_SOLVER_H__
#define __MULTI_CONTACT_MOTION_SOLVER_H__

#include <HumanoidMotionControlCore/HumanoidMotionControlCore.h>
#include <Interface/TaskExecutionManager.h>
#include <Model/HumanoidBody.h>
#include <Model/SharedData.h>
#include <Model/State.h>
#include <Util/EigenUtil.h>

#include <mc_rbdyn/RobotLoader.h>
#include <mc_solver/QPSolver.h>
#include <mc_solver/ConstraintSet.h>
#include <mc_solver/ContactConstraint.h>
#include <Math/Filter/ComplementaryFilter.h>
#include "Tasks/Task.h"

#include <thread>

namespace humanoid_motion_control_core {
  class HumanoidMotionControlCore;
}

namespace multi_contact_motion_solver {

  class MultiContactMotionSolver : public common_interface::TaskExecutionManager {

  public:
  
    MultiContactMotionSolver(common_interface::TaskExecutionManager* parentManager,
                             const char* name = "multi-contact-motion-solver",
                             std::list<std::string>* cmd_init = NULL);
    ~MultiContactMotionSolver();
    
    bool isReadyToActivate(void);
    bool isReadyToDeactivate(void);
    bool onActivate(void);
    bool onExecute(void);
  
    // Shared InPort Data (Methods)

    SharedDataBase* qInit(void) {return &m_qInit_shared;}
    SharedDataBase* waistPoseInit(void) {return &m_waistPoseInit_shared;}

    SharedDataBase* qOmitRefIn(void) {return &m_qOmitRefIn_shared;}
    
    SharedDataBase* tauIn(void) {return &m_tauIn_shared;}
    SharedDataBase* qIn(void)   {return &m_qIn_shared;}
    
    SharedDataBase* waistPIn(void)    {return &m_waistPIn_shared;}
    SharedDataBase* waistRIn(void)    {return &m_waistRIn_shared;}
    SharedDataBase* waistPoseIn(void) {return &m_waistPoseIn_shared;}
    
    SharedDataBase* waistLinVelIn(void)         {return &m_waistLinVelIn_shared;}
    SharedDataBase* waistAngVelIn(void)         {return &m_waistAngVelIn_shared;}
    SharedDataBase* waistGeneralizedVelIn(void) {return &m_waistGeneralizedVelIn_shared;}
    SharedDataBase* waistLinAccIn(void)         {return &m_waistLinAccIn_shared;}
    SharedDataBase* waistStateIn(void)          {return &m_waistStateIn_shared;}
    
    SharedDataBase* rFootWrenchSensorIn(void) {return &m_rFootWrenchSensorIn_shared;}
    SharedDataBase* rFootForceSensorIn(void)  {return &m_rFootForceSensorIn_shared;}
    SharedDataBase* rFootMomentSensorIn(void) {return &m_rFootMomentSensorIn_shared;}
    SharedDataBase* lFootWrenchSensorIn(void) {return &m_lFootWrenchSensorIn_shared;}
    SharedDataBase* lFootForceSensorIn(void)  {return &m_lFootForceSensorIn_shared;}
    SharedDataBase* lFootMomentSensorIn(void) {return &m_lFootMomentSensorIn_shared;}
    
    SharedDataBase* rHandWrenchSensorIn(void) {return &m_rHandWrenchSensorIn_shared;}
    SharedDataBase* rHandForceSensorIn(void)  {return &m_rHandForceSensorIn_shared;}
    SharedDataBase* rHandMomentSensorIn(void) {return &m_rHandMomentSensorIn_shared;}
    SharedDataBase* lHandWrenchSensorIn(void) {return &m_lHandWrenchSensorIn_shared;}
    SharedDataBase* lHandForceSensorIn(void)  {return &m_lHandForceSensorIn_shared;}
    SharedDataBase* lHandMomentSensorIn(void) {return &m_lHandMomentSensorIn_shared;}
  
    // Shared OutPort Data (Methods)

    SharedDataBase* torqueModeOut(void) {return &m_torqueModeOut_shared;}
    SharedDataBase* tauRefOut(void) {return &m_tauRefOut_shared;}

    SharedDataBase* tauPOut(void) {return &m_tauPOut_shared;}
    SharedDataBase* gammaDOut(void) {return &m_gammaDOut_shared;}

    SharedDataBase* qRefOut(void) {return &m_qRefOut_shared;}
    SharedDataBase* dqRefOut(void) {return &m_dqRefOut_shared;}
    SharedDataBase* ddqRefOut(void) {return &m_ddqRefOut_shared;}

    SharedDataBase* waistStateRefOut(void) {return &m_waistStateRefOut_shared;}
    SharedDataBase* waistPRefOut(void) {return &m_waistPRefOut_shared;}
    SharedDataBase* waistRRefOut(void) {return &m_waistRRefOut_shared;}
    
    SharedDataBase* rFootWrenchRefOut(void) {return &m_rFootWrenchRefOut_shared;}
    SharedDataBase* lFootWrenchRefOut(void) {return &m_lFootWrenchRefOut_shared;}
    SharedDataBase* rHandWrenchRefOut(void) {return &m_rHandWrenchRefOut_shared;}
    SharedDataBase* lHandWrenchRefOut(void) {return &m_lHandWrenchRefOut_shared;}
    
    SharedDataBase* rFootWrenchHatOut(void) {return &m_rFootWrenchHatOut_shared;}
    SharedDataBase* lFootWrenchHatOut(void) {return &m_lFootWrenchHatOut_shared;}
    SharedDataBase* rHandWrenchHatOut(void) {return &m_rHandWrenchHatOut_shared;}
    SharedDataBase* lHandWrenchHatOut(void) {return &m_lHandWrenchHatOut_shared;}
    
  protected:
    
    humanoid_motion_control_core::HumanoidMotionControlCore* hmc;
    motion_generator::HumanoidBodyPtr m_body;

    // Shared InPort Data (Variables)

    hrp::dvector* m_qInit;
    SharedData<hrp::dvector*> m_qInit_shared;
    StatePosAtt* m_waistPoseInit;
    SharedData<StatePosAtt*> m_waistPoseInit_shared;

    hrp::dvector* m_qOmitRefIn;
    SharedData<hrp::dvector*> m_qOmitRefIn_shared;

    hrp::dvector* m_tauIn;
    SharedData<hrp::dvector*> m_tauIn_shared;
    hrp::dvector* m_qIn;
    SharedData<hrp::dvector*> m_qIn_shared;

    hrp::Vector3* m_waistPIn;
    SharedData<hrp::Vector3*> m_waistPIn_shared;
    hrp::Matrix33* m_waistRIn;
    SharedData<hrp::Matrix33*> m_waistRIn_shared;
    StatePosAtt* m_waistPoseIn;
    SharedData<StatePosAtt*> m_waistPoseIn_shared;

    hrp::Vector3* m_waistLinVelIn;
    SharedData<hrp::Vector3*> m_waistLinVelIn_shared;
    hrp::Vector3* m_waistAngVelIn;
    SharedData<hrp::Vector3*> m_waistAngVelIn_shared;
    hrp::dvector6* m_waistGeneralizedVelIn;
    SharedData<hrp::dvector6*> m_waistGeneralizedVelIn_shared;
    hrp::Vector3* m_waistLinAccIn;
    SharedData<hrp::Vector3*> m_waistLinAccIn_shared;
    State* m_waistStateIn;
    SharedData<State*> m_waistStateIn_shared;
    
    hrp::dvector6* m_rFootWrenchSensorIn;
    SharedData<hrp::dvector6*> m_rFootWrenchSensorIn_shared;
    hrp::Vector3* m_rFootForceSensorIn;
    SharedData<hrp::Vector3*> m_rFootForceSensorIn_shared;
    hrp::Vector3* m_rFootMomentSensorIn;
    SharedData<hrp::Vector3*> m_rFootMomentSensorIn_shared;

    hrp::dvector6* m_lFootWrenchSensorIn;
    SharedData<hrp::dvector6*> m_lFootWrenchSensorIn_shared;
    hrp::Vector3* m_lFootForceSensorIn;
    SharedData<hrp::Vector3*> m_lFootForceSensorIn_shared;
    hrp::Vector3* m_lFootMomentSensorIn;
    SharedData<hrp::Vector3*> m_lFootMomentSensorIn_shared;
    
    hrp::dvector6* m_rHandWrenchSensorIn;
    SharedData<hrp::dvector6*> m_rHandWrenchSensorIn_shared;
    hrp::Vector3* m_rHandForceSensorIn;
    SharedData<hrp::Vector3*> m_rHandForceSensorIn_shared;
    hrp::Vector3* m_rHandMomentSensorIn;
    SharedData<hrp::Vector3*> m_rHandMomentSensorIn_shared;

    hrp::dvector6* m_lHandWrenchSensorIn;
    SharedData<hrp::dvector6*> m_lHandWrenchSensorIn_shared;
    hrp::Vector3* m_lHandForceSensorIn;
    SharedData<hrp::Vector3*> m_lHandForceSensorIn_shared;
    hrp::Vector3* m_lHandMomentSensorIn;
    SharedData<hrp::Vector3*> m_lHandMomentSensorIn_shared;
    
    // Shared OutPort Data (Variables)

    std::vector<bool> m_torqueModeOut;
    SharedData<std::vector<bool>> m_torqueModeOut_shared;
    hrp::dvector m_tauRefOut;
    SharedData<hrp::dvector> m_tauRefOut_shared;

    hrp::dvector m_tauPOut;
    SharedData<hrp::dvector> m_tauPOut_shared;
    hrp::dvector m_gammaDOut;
    SharedData<hrp::dvector> m_gammaDOut_shared;
    
    hrp::dvector m_qRefOut;
    SharedData<hrp::dvector> m_qRefOut_shared;
    hrp::dvector m_dqRefOut;
    SharedData<hrp::dvector> m_dqRefOut_shared;
    hrp::dvector m_ddqRefOut;
    SharedData<hrp::dvector> m_ddqRefOut_shared;

    State m_waistStateRefOut;
    SharedData<State>         m_waistStateRefOut_shared;
    SharedData<hrp::Vector3>  m_waistPRefOut_shared;
    SharedData<hrp::Matrix33> m_waistRRefOut_shared;
    
    hrp::dvector6 m_rFootWrenchRefOut;
    SharedData<hrp::dvector6> m_rFootWrenchRefOut_shared;
    hrp::dvector6 m_lFootWrenchRefOut;
    SharedData<hrp::dvector6> m_lFootWrenchRefOut_shared;
    hrp::dvector6 m_rHandWrenchRefOut;
    SharedData<hrp::dvector6> m_rHandWrenchRefOut_shared;
    hrp::dvector6 m_lHandWrenchRefOut;
    SharedData<hrp::dvector6> m_lHandWrenchRefOut_shared;

    hrp::dvector6 m_rFootWrenchHatOut;
    SharedData<hrp::dvector6> m_rFootWrenchHatOut_shared;
    hrp::dvector6 m_lFootWrenchHatOut;
    SharedData<hrp::dvector6> m_lFootWrenchHatOut_shared;
    hrp::dvector6 m_rHandWrenchHatOut;
    SharedData<hrp::dvector6> m_rHandWrenchHatOut_shared;
    hrp::dvector6 m_lHandWrenchHatOut;
    SharedData<hrp::dvector6> m_lHandWrenchHatOut_shared;
    
    // Solver Related

    bool m_running;
    bool m_init;

    mc_solver::QPSolver* m_solver;
    std::mutex m_mutex;

  public:

    const motion_generator::HumanoidBodyPtr body();
    const std::vector<motion_generator::LinkInr> & linkInr();
    mc_rbdyn::Robots & robots();
    mc_rbdyn::Robot  & robot();
    mc_rbdyn::RobotModule & robot_module();
    const torque_control::TorqueFeedbackTerm::TorqueControlType torqueControlType();
    const std::shared_ptr<torque_control::TorqueFeedbackTerm> fbTerm() const;

    mc_solver::QPSolver & solver();

    template<typename Derived>
    inline bool containsNan(const Eigen::MatrixBase<Derived>& M)
    {
      return (isnan(M.array())).count() > 0;
    }

    void addTask(tasks::qp::Task* task);
    void addTask(mc_tasks::MetaTask* task);

    void removeTask(tasks::qp::Task* task);
    void removeTask(mc_tasks::MetaTask* task);
    
    void addConstraint(tasks::qp::ConstraintFunction<tasks::qp::Equality>* constraint);
    void addConstraint(tasks::qp::ConstraintFunction<tasks::qp::Inequality>* constraint);
    void addConstraint(tasks::qp::ConstraintFunction<tasks::qp::GenInequality>* constraint);
    void addConstraint(tasks::qp::ConstraintFunction<tasks::qp::Bound>* constraint);
    void addConstraintSet(mc_solver::ConstraintSet & constraint_set);

    void removeConstraint(tasks::qp::ConstraintFunction<tasks::qp::Equality>* constraint);
    void removeConstraint(tasks::qp::ConstraintFunction<tasks::qp::Inequality>* constraint);
    void removeConstraint(tasks::qp::ConstraintFunction<tasks::qp::GenInequality>* constraint);
    void removeConstraint(tasks::qp::ConstraintFunction<tasks::qp::Bound>* constraint);
    void removeConstraintSet(mc_solver::ConstraintSet & constraint_set);

    void setContacts(const std::vector<mc_rbdyn::Contact> & contacts);
    
  private:
    
    std::shared_ptr<mc_rbdyn::RobotModule> m_robot_module;
    std::map<std::string, sva::ForceVecd> m_wrenches;
    std::vector<motion_generator::LinkInr> m_linkInr;

    std::vector<std::string> m_omitted_joints;

    torque_control::TorqueFeedbackTerm::TorqueControlType m_torqueControlType;
    torque_control::IntegralTerm::IntegralTermType m_intglTermType;
    torque_control::IntegralTerm::VelocityGainType m_velGainType;
    double m_beta, m_lambda, m_mu, m_sigma, m_cis;

    hrp::dvector m_alphaVec_hat_prev;
    filter::ComplementaryFilter m_CompFilter;
    bool m_first, m_failed;

    bool init(const std::vector<double> & initq);
    bool init(const std::vector<double> & initq, const std::array<double, 7> & initAttitude);
    bool run();
    void reset(const std::vector<std::vector<double>> & q);

    void addRotorInertia();
    void addRotorInertia(mc_rbdyn::Robot& robot);

    Eigen::Vector6d computeWrench(std::string linkName, const tasks::qp::BilateralContact & contact, Eigen::VectorXd lambda, unsigned int pos);

    void publish_reference_wrenches(const mc_solver::QPResultMsg & res);
    void publish_measured_wrenches();

    void enable_feedback_torque_mode(bool flag);

    /** \brief Enables (flag = 1) / disables (flag = 0) the motion solver.
     *
     *  Usage:
     *  ":enable-solver flag"
     */
    bool cmd_enable_solver(std::istringstream& i_strm, std::ostringstream& o_strm);


    /** \brief Selects the joints (identified by joint-name_i) that will be omitted
     *         by the solver (and the dynamics).
     *         If no joint-name is specified (possible), no joint will be omitted.
     *
     *  Usage:
     *  ":omit-joints-in-solver joint-name_a joint-name_b ..."
     *  ":omit-joints-in-solver"
     */
    bool cmd_omit_joints_in_solver(std::istringstream& i_strm, std::ostringstream& o_strm);

    /** \brief Enables (flag = 1) / disables (flag = 0) the feedback / torque mode
     *         on the joints that are not omitted by the solver.
     *
     *  Usage:
     *  ":enable-feedback-torque-mode flag"
     */    
    bool cmd_enable_feedback_torque_mode(std::istringstream& i_strm, std::ostringstream& o_strm);
    
    /** \brief
     *
     *  Selects the torque control \f$ type \f$, which can be
     *  a) None
     *  b) IntegralTerm
     *  c) PassivityPIDTerm
     *  
     *  If IntegralTerm is specified, the integral term type ( \f$ intglTermType \f$ ) :
     *  \f$ L \f$ (\f$ P = L s \f$) can be selected as
     *  None:           L = 0       (same effect as none for torque control type)
     *  Simple:         L = K
     *  PassivityBased: L = (C + K)
     *
     *  Where the velocity gain is specified by a value for \f$ lambda \f$ and
     *  a \f$ velGainType \$ as
     *  Diagonal:       K = lambda * eye
     *  MassMatrix:     K = lambda * M
     *
     *  If PassivityPIDTerm is specified, its parameters \f$ beta \f$, \f$ lambda \f$,
     *  \f$ mu \f$, \f$ sigma \f$ and \f$ cis \f$ are read next, such that
     *  P = Kv s + Kp e + Ki E
     *
     *  Where
     *  Kv = lambda * M + C + Ka
     *  Kp = mu * M + lambda * (C + Ka) + L
     *  Ki = mu * (C + Ka) + cis * lambda * L
     *
     *  And
     *  Ka = beta * M
     *  L  = sigma * diag(M)
     *
     *  Usage:
     *  ":select-torque-control-type None"
     *  ":select-torque-control-type IntegralTerm integralTermType lambda velGainType"
     *  ":select-torque-control-type PassivityPIDTerm beta lambda mu sigma cis"
     */
    bool cmd_select_torque_control_type(std::istringstream& i_strm, std::ostringstream& o_strm);
  };

}

#endif // __MULTI_CONTACT_MOTION_SOLVER_H__
