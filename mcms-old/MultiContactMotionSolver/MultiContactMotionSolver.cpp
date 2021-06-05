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

#include "MultiContactMotionSolver.h"
#include <MultiContactMotionSolver/Constraints/TorqueConstraint.h>
#include <Math/MathFunction.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/FA.h>
#include <RBDyn/FD.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/TorqueFeedbackTerm.h>
#include <time.h>

using namespace hrp;
using namespace common_interface;
using namespace humanoid_motion_control_core;
using namespace motion_generator;
using namespace multi_contact_motion_solver;

MultiContactMotionSolver::MultiContactMotionSolver(TaskExecutionManager* parentManager,
                                                   const char* name,
                                                   std::list<std::string>* cmd_init) :

  TaskExecutionManager(parentManager, name, cmd_init),
  hmc(dynamic_cast<HumanoidMotionControlCore*>(findRootTaskExecutionManager(parentManager))),
  m_body(new HumanoidBody(*hmc->getBody())),

  // Shared InPort Data (Initializations)

  m_qInit(NULL),
  m_qInit_shared(m_qInit),
  m_waistPoseInit(NULL),
  m_waistPoseInit_shared(m_waistPoseInit),

  m_qOmitRefIn(NULL),
  m_qOmitRefIn_shared(m_qOmitRefIn),
  
  m_tauIn(NULL),
  m_tauIn_shared(m_tauIn),
  m_qIn(NULL),
  m_qIn_shared(m_qIn),

  m_waistPIn(NULL),
  m_waistPIn_shared(m_waistPIn),
  m_waistRIn(NULL),
  m_waistRIn_shared(m_waistRIn),
  m_waistPoseIn(NULL),
  m_waistPoseIn_shared(m_waistPoseIn),

  m_waistLinVelIn(NULL),
  m_waistLinVelIn_shared(m_waistLinVelIn),
  m_waistAngVelIn(NULL),
  m_waistAngVelIn_shared(m_waistAngVelIn),
  m_waistGeneralizedVelIn(NULL),
  m_waistGeneralizedVelIn_shared(m_waistGeneralizedVelIn),
  m_waistLinAccIn(NULL),
  m_waistLinAccIn_shared(m_waistLinAccIn),
  m_waistStateIn(NULL),
  m_waistStateIn_shared(m_waistStateIn),

  m_rFootWrenchSensorIn(NULL),
  m_rFootWrenchSensorIn_shared(m_rFootWrenchSensorIn),
  m_rFootForceSensorIn(NULL),
  m_rFootForceSensorIn_shared(m_rFootForceSensorIn),
  m_rFootMomentSensorIn(NULL),
  m_rFootMomentSensorIn_shared(m_rFootMomentSensorIn),
  
  m_lFootWrenchSensorIn(NULL),
  m_lFootWrenchSensorIn_shared(m_lFootWrenchSensorIn),
  m_lFootForceSensorIn(NULL),
  m_lFootForceSensorIn_shared(m_lFootForceSensorIn),
  m_lFootMomentSensorIn(NULL),
  m_lFootMomentSensorIn_shared(m_lFootMomentSensorIn),
  
  m_rHandWrenchSensorIn(NULL),
  m_rHandWrenchSensorIn_shared(m_rHandWrenchSensorIn),
  m_rHandForceSensorIn(NULL),
  m_rHandForceSensorIn_shared(m_rHandForceSensorIn),
  m_rHandMomentSensorIn(NULL),
  m_rHandMomentSensorIn_shared(m_rHandMomentSensorIn),
  
  m_lHandWrenchSensorIn(NULL),
  m_lHandWrenchSensorIn_shared(m_lHandWrenchSensorIn),
  m_lHandForceSensorIn(NULL),
  m_lHandForceSensorIn_shared(m_lHandForceSensorIn),
  m_lHandMomentSensorIn(NULL),
  m_lHandMomentSensorIn_shared(m_lHandMomentSensorIn),
  
  // Shared OutPort Data (Initializations)

  m_torqueModeOut(std::vector<bool>(m_body->numJoints(), false)),
  m_torqueModeOut_shared(m_torqueModeOut),

  m_tauRefOut(dvector::Zero(m_body->numJoints())),
  m_tauRefOut_shared(m_tauRefOut),

  m_tauPOut(dvector::Zero(39)), // Rafa, change this later
  m_tauPOut_shared(m_tauPOut),

  m_gammaDOut(dvector::Zero(39)), // Rafa, change this later
  m_gammaDOut_shared(m_gammaDOut),
  
  m_qRefOut(dvector::Zero(m_body->numJoints())),
  m_qRefOut_shared(m_qRefOut),

  m_dqRefOut(dvector::Zero(m_body->numJoints())),
  m_dqRefOut_shared(m_dqRefOut),

  m_ddqRefOut(dvector::Zero(m_body->numJoints())),
  m_ddqRefOut_shared(m_ddqRefOut),

  m_waistStateRefOut(),
  m_waistStateRefOut_shared(m_waistStateRefOut),
  m_waistPRefOut_shared(m_waistStateRefOut.P),
  m_waistRRefOut_shared(m_waistStateRefOut.R),
  
  m_rFootWrenchRefOut(dvector6::Zero()),
  m_rFootWrenchRefOut_shared(m_rFootWrenchRefOut),

  m_lFootWrenchRefOut(dvector6::Zero()),
  m_lFootWrenchRefOut_shared(m_lFootWrenchRefOut),

  m_rHandWrenchRefOut(dvector6::Zero()),
  m_rHandWrenchRefOut_shared(m_rHandWrenchRefOut),

  m_lHandWrenchRefOut(dvector6::Zero()),
  m_lHandWrenchRefOut_shared(m_lHandWrenchRefOut),

  m_rFootWrenchHatOut(dvector6::Zero()),
  m_rFootWrenchHatOut_shared(m_rFootWrenchHatOut),

  m_lFootWrenchHatOut(dvector6::Zero()),
  m_lFootWrenchHatOut_shared(m_lFootWrenchHatOut),

  m_rHandWrenchHatOut(dvector6::Zero()),
  m_rHandWrenchHatOut_shared(m_rHandWrenchHatOut),

  m_lHandWrenchHatOut(dvector6::Zero()),
  m_lHandWrenchHatOut_shared(m_lHandWrenchHatOut),
  
  // Solver Related

  m_omitted_joints(std::vector<std::string>(0)),
  m_running(false),
  m_init(false),

  m_torqueControlType(torque_control::TorqueFeedbackTerm::None),
  m_intglTermType(torque_control::IntegralTerm::None),
  m_velGainType(torque_control::IntegralTerm::Diagonal),

  m_beta(0.0),
  m_lambda(0.0),
  m_mu(0.0),
  m_sigma(0.0),
  m_cis(0.0),

  m_CompFilter(0.95), // 0.1
  m_first(true),
  m_failed(false),
  
  m_solver(NULL)
{
  // Shared InPort Data (Registrations)

  registerInPort("qInit", (sharedFuncPtr) &MultiContactMotionSolver::qInit);
  registerInPort("waistPoseInit", (sharedFuncPtr) &MultiContactMotionSolver::waistPoseInit);

  registerInPort("qOmitRefIn", (sharedFuncPtr) &MultiContactMotionSolver::qOmitRefIn);
  
  registerInPort("tauIn", (sharedFuncPtr) &MultiContactMotionSolver::tauIn);
  registerInPort("qIn", (sharedFuncPtr) &MultiContactMotionSolver::qIn);
  
  registerInPort("waistPIn", (sharedFuncPtr) &MultiContactMotionSolver::waistPIn);
  registerInPort("waistRIn", (sharedFuncPtr) &MultiContactMotionSolver::waistRIn);
  registerInPort("waistPoseIn", (sharedFuncPtr) &MultiContactMotionSolver::waistPoseIn);
  
  registerInPort("waistLinVelIn", (sharedFuncPtr) &MultiContactMotionSolver::waistLinVelIn);
  registerInPort("waistAngVelIn", (sharedFuncPtr) &MultiContactMotionSolver::waistAngVelIn);
  registerInPort("waistGeneralizedVelIn", (sharedFuncPtr) &MultiContactMotionSolver::waistGeneralizedVelIn);
  registerInPort("waistLinAccIn", (sharedFuncPtr) &MultiContactMotionSolver::waistLinAccIn);
  registerInPort("waistStateIn", (sharedFuncPtr) &MultiContactMotionSolver::waistStateIn);

  registerInPort("rFootWrenchSensorIn", (sharedFuncPtr) &MultiContactMotionSolver::rFootWrenchSensorIn);
  registerInPort("rFootForceSensorIn", (sharedFuncPtr) &MultiContactMotionSolver::rFootForceSensorIn);
  registerInPort("rFootMomentSensorIn", (sharedFuncPtr) &MultiContactMotionSolver::rFootMomentSensorIn);
  registerInPort("lFootWrenchSensorIn", (sharedFuncPtr) &MultiContactMotionSolver::lFootWrenchSensorIn);
  registerInPort("lFootForceSensorIn", (sharedFuncPtr) &MultiContactMotionSolver::lFootForceSensorIn);
  registerInPort("lFootMomentSensorIn", (sharedFuncPtr) &MultiContactMotionSolver::lFootMomentSensorIn);
  
  registerInPort("rHandWrenchSensorIn", (sharedFuncPtr) &MultiContactMotionSolver::rHandWrenchSensorIn);
  registerInPort("rHandForceSensorIn", (sharedFuncPtr) &MultiContactMotionSolver::rHandForceSensorIn);
  registerInPort("rHandMomentSensorIn", (sharedFuncPtr) &MultiContactMotionSolver::rHandMomentSensorIn);
  registerInPort("lHandWrenchSensorIn", (sharedFuncPtr) &MultiContactMotionSolver::lHandWrenchSensorIn);
  registerInPort("lHandForceSensorIn", (sharedFuncPtr) &MultiContactMotionSolver::lHandForceSensorIn);
  registerInPort("lHandMomentSensorIn", (sharedFuncPtr) &MultiContactMotionSolver::lHandMomentSensorIn);
  
  // Shared OutPort Data (Registrations)

  registerOutPort("torqueModeOut", (sharedFuncPtr) &MultiContactMotionSolver::torqueModeOut);
  registerOutPort("tauRefOut", (sharedFuncPtr) &MultiContactMotionSolver::tauRefOut);
  
  registerOutPort("tauPOut", (sharedFuncPtr) &MultiContactMotionSolver::tauPOut);
  registerOutPort("gammaDOut", (sharedFuncPtr) &MultiContactMotionSolver::gammaDOut);

  registerOutPort("qRefOut", (sharedFuncPtr) &MultiContactMotionSolver::qRefOut);
  registerOutPort("dqRefOut", (sharedFuncPtr) &MultiContactMotionSolver::dqRefOut);
  registerOutPort("ddqRefOut", (sharedFuncPtr) &MultiContactMotionSolver::ddqRefOut);

  registerOutPort("waistStateRefOut", (sharedFuncPtr) &MultiContactMotionSolver::waistStateRefOut);
  registerOutPort("waistPRefOut", (sharedFuncPtr) &MultiContactMotionSolver::waistPRefOut);
  registerOutPort("waistRRefOut", (sharedFuncPtr) &MultiContactMotionSolver::waistRRefOut);

  registerOutPort("rFootWrenchRefOut", (sharedFuncPtr) &MultiContactMotionSolver::rFootWrenchRefOut);
  registerOutPort("lFootWrenchRefOut", (sharedFuncPtr) &MultiContactMotionSolver::lFootWrenchRefOut);
  registerOutPort("rHandWrenchRefOut", (sharedFuncPtr) &MultiContactMotionSolver::rHandWrenchRefOut);
  registerOutPort("lHandWrenchRefOut", (sharedFuncPtr) &MultiContactMotionSolver::lHandWrenchRefOut);
  
  registerOutPort("rFootWrenchHatOut", (sharedFuncPtr) &MultiContactMotionSolver::rFootWrenchHatOut);
  registerOutPort("lFootWrenchHatOut", (sharedFuncPtr) &MultiContactMotionSolver::lFootWrenchHatOut);
  registerOutPort("rHandWrenchHatOut", (sharedFuncPtr) &MultiContactMotionSolver::rHandWrenchHatOut);
  registerOutPort("lHandWrenchHatOut", (sharedFuncPtr) &MultiContactMotionSolver::lHandWrenchHatOut);
  
  // Method Functions

  registerMethodFunction(":enable-solver",
                         (methodFuncPtr) &MultiContactMotionSolver::cmd_enable_solver);
  registerMethodFunction(":omit-joints-in-solver",
                         (methodFuncPtr) &MultiContactMotionSolver::cmd_omit_joints_in_solver);
  registerMethodFunction(":enable-feedback-torque-mode",
                         (methodFuncPtr) &MultiContactMotionSolver::cmd_enable_feedback_torque_mode);
  /*
  registerMethodFunction(":enable-feedback",
                         (methodFuncPtr) &MultiContactMotionSolver::cmd_enable_feedback);
  registerMethodFunction(":set-torque-mode-all-joints",
                         (methodFuncPtr) &MultiContactMotionSolver::cmd_set_torque_mode_all_joints);
  registerMethodFunction(":set-torque-mode-some-joints",
                         (methodFuncPtr) &MultiContactMotionSolver::cmd_set_torque_mode_some_joints);
  */
  registerMethodFunction(":select-torque-control-type",
                         (methodFuncPtr) &MultiContactMotionSolver::cmd_select_torque_control_type);

  initialCommands(cmd_init);
  
  // Solver Related

  m_body->initComInertia(m_linkInr);
  
  std::string robot_name = m_body->modelName();
  
  if (mc_rbdyn::RobotLoader::has_robot(robot_name)) {
    try {
      
      m_robot_module = mc_rbdyn::RobotLoader::get_robot_module(robot_name);

      if (!m_omitted_joints.empty()) {
        
        m_robot_module->mbg = m_robot_module->mbg.fixRevJoints(m_robot_module->mbg, m_omitted_joints);
        m_robot_module->mb  = m_robot_module->mbg.makeMultiBody(m_robot_module->mb.body(0).name(), false);
        m_robot_module->mbc = rbd::MultiBodyConfig(m_robot_module->mb);
        m_robot_module->mbc.zero(m_robot_module->mb);
        
        for (std::string & joint : m_omitted_joints)
          // 0 : lower joint limits;    1 : upper joint limits
          // 2 : lower velocity limits; 3 : upper velocity limits
          // 4 : lower torque limits;   5 : upper torque limits
          for (size_t b = 0; b < 6; b++)
            m_robot_module->_bounds[b].erase(joint);
      }
    }
    catch (const mc_rtc::LoaderException & exc) {
      LOG_ERROR("Failed to create " << robot_name << " to use as a main robot");
      throw std::runtime_error("Failed to create robot");
    }
  }
  else {
    LOG_ERROR("Trying to use " << robot_name << " as main robot but this robot cannot be loaded");
    throw std::runtime_error("Main robot not available");
  }
  
  // ground_module will correspond to the entire environment
  std::shared_ptr<mc_rbdyn::RobotModule> ground_module = mc_rbdyn::RobotLoader::get_robot_module("env", std::string(mc_rtc::MC_ENV_DESCRIPTION_PATH), std::string("ground"));
  
  std::vector<std::shared_ptr<mc_rbdyn::RobotModule>> robots_modules = {m_robot_module, ground_module};

  std::shared_ptr<mc_rbdyn::Robots> robots = mc_rbdyn::loadRobots(robots_modules);

  for (mc_rbdyn::Robot & robot : robots->robots()) {
    robot.mbc().gravity = Vector3(0.0, 0.0, 9.81);
    rbd::forwardKinematics(robot.mb(), robot.mbc());
    rbd::forwardVelocity(robot.mb(), robot.mbc());
    rbd::forwardAcceleration(robot.mb(), robot.mbc());
  }
  
  addRotorInertia(robots->robot());

  Eigen::VectorXd torqueL = Eigen::VectorXd(robots->robot().mb().nrDof());
  Eigen::VectorXd torqueU = Eigen::VectorXd(robots->robot().mb().nrDof());
  
  rbd::paramToVector(robots->robot().tl(), torqueL);
  rbd::paramToVector(robots->robot().tu(), torqueU);

  if (m_torqueControlType == torque_control::TorqueFeedbackTerm::PassivityPIDTerm)
    m_solver = new mc_solver::PassivityPIDTerm_QPSolver(robots, m_dt, m_beta, m_lambda, m_mu, m_sigma, m_cis);
  else {
    // m_solver = new mc_solver::IntglTermAntiWindup_QPSolver(robots, m_dt, m_intglTermType, m_velGainType, m_lambda, torqueL, torqueU, 1E3, 0.2);
    m_solver = new mc_solver::IntglTerm_QPSolver(robots, m_dt, m_intglTermType, m_velGainType, m_lambda);
  }
  
  // Force/Torque sensor names are harcoded in mc_rtc 
  m_wrenches["RightFootForceSensor"] = sva::ForceVecd(Vector3(0, 0, 0), Vector3(0, 0, 0));
  m_wrenches["LeftFootForceSensor"]  = sva::ForceVecd(Vector3(0, 0, 0), Vector3(0, 0, 0));
  m_wrenches["RightHandForceSensor"] = sva::ForceVecd(Vector3(0, 0, 0), Vector3(0, 0, 0));
  m_wrenches["LeftHandForceSensor"]  = sva::ForceVecd(Vector3(0, 0, 0), Vector3(0, 0, 0));

  m_alphaVec_hat_prev = dvector::Zero(robot().mb().nrDof());
  m_CompFilter.reset(robot().mb().nrDof());
}

MultiContactMotionSolver::~MultiContactMotionSolver()
{
  if (m_solver)
    delete m_solver;
}

bool MultiContactMotionSolver::isReadyToActivate(void)
{
  return TaskExecutionManager::isReadyToActivate();
}

bool MultiContactMotionSolver::isReadyToDeactivate(void)
{
  return TaskExecutionManager::isReadyToDeactivate();
}

bool MultiContactMotionSolver::onActivate(void)
{
  if (!hmc->checkReservedInPort(qIn())) {
    std::cout << "ERROR!!!: " << qIn()->getPortName() << " is not connected in "
              << getTaskName() << std::endl;
    return false;
  }

  if (!hmc->checkReservedInPort(waistStateIn())) {

    if (!hmc->checkReservedInPort(waistPoseIn())) {
  
      if (!hmc->checkReservedInPort(waistPIn())) {
        std::cout << "ERROR!!!: "
                  << waistPIn()->getPortName() << " (or "
                  << waistPoseIn()->getPortName() << " or "
                  << waistStateIn()->getPortName() << ") is not connected in "
                  << getTaskName() << std::endl;
        return false;
      }
  
      if (!hmc->checkReservedInPort(waistRIn())) {
        std::cout << "ERROR!!!: "
                  << waistRIn()->getPortName() << " (or "
                  << waistPoseIn()->getPortName() << " or "
                  << waistStateIn()->getPortName() << ") is not connected in "
                  << getTaskName() << std::endl;
        return false;
      }
    }

    if (!hmc->checkReservedInPort(waistGeneralizedVelIn())) {

      if (!hmc->checkReservedInPort(waistLinVelIn())) {
        std::cout << "ERROR!!!: "
                  << waistLinVelIn()->getPortName() << " (or "
                  << waistGeneralizedVelIn()->getPortName() << " or "
                  << waistStateIn()->getPortName() << ") is not connected in "
                  << getTaskName() << std::endl;
        return false;
      }

      if (!hmc->checkReservedInPort(waistAngVelIn())) {
        std::cout << "ERROR!!!: "
                  << waistAngVelIn()->getPortName() << " (or "
                  << waistGeneralizedVelIn()->getPortName() << " or "
                  << waistStateIn()->getPortName() << ") is not connected in "
                  << getTaskName() << std::endl;
        return false;
      }
    }

    if (!hmc->checkReservedInPort(waistLinAccIn())) {
      std::cout << "ERROR!!!: "
                << waistLinAccIn()->getPortName() << " (or "
                << waistStateIn()->getPortName() << ") is not connected in "
                << getTaskName() << std::endl;
      return false;
    }
  }
  
  if (!hmc->checkReservedInPort(rFootWrenchSensorIn())) {

    if (!hmc->checkReservedInPort(rFootForceSensorIn())) {
      std::cout << "ERROR!!!: "
                << rFootForceSensorIn()->getPortName() << " (or "
                << rFootWrenchSensorIn()->getPortName() << ") is not connected in "
                << getTaskName() << std::endl;
      return false;
    }

    if (!hmc->checkReservedInPort(rFootMomentSensorIn())) {
      std::cout << "ERROR!!!: "
                << rFootMomentSensorIn()->getPortName() << " (or "
                << rFootWrenchSensorIn()->getPortName() << ") is not connected in "
                << getTaskName() << std::endl;
      return false;
    }    
  }

  if (!hmc->checkReservedInPort(lFootWrenchSensorIn())) {

    if (!hmc->checkReservedInPort(lFootForceSensorIn())) {
      std::cout << "ERROR!!!: "
                << lFootForceSensorIn()->getPortName() << " (or "
                << lFootWrenchSensorIn()->getPortName() << ") is not connected in "
                << getTaskName() << std::endl;
      return false;
    }

    if (!hmc->checkReservedInPort(lFootMomentSensorIn())) {
      std::cout << "ERROR!!!: "
                << lFootMomentSensorIn()->getPortName() << " (or "
                << lFootWrenchSensorIn()->getPortName() << ") is not connected in "
                << getTaskName() << std::endl;
      return false;
    }    
  }

  if (!hmc->checkReservedInPort(rHandWrenchSensorIn())) {

    if (!hmc->checkReservedInPort(rHandForceSensorIn())) {
      std::cout << "ERROR!!!: "
                << rHandForceSensorIn()->getPortName() << " (or "
                << rHandWrenchSensorIn()->getPortName() << ") is not connected in "
                << getTaskName() << std::endl;
      return false;
    }

    if (!hmc->checkReservedInPort(rHandMomentSensorIn())) {
      std::cout << "ERROR!!!: "
                << rHandMomentSensorIn()->getPortName() << " (or "
                << rHandWrenchSensorIn()->getPortName() << ") is not connected in "
                << getTaskName() << std::endl;
      return false;
    }    
  }

  if (!hmc->checkReservedInPort(lHandWrenchSensorIn())) {

    if (!hmc->checkReservedInPort(lHandForceSensorIn())) {
      std::cout << "ERROR!!!: "
                << lHandForceSensorIn()->getPortName() << " (or "
                << lHandWrenchSensorIn()->getPortName() << ") is not connected in "
                << getTaskName() << std::endl;
      return false;
    }

    if (!hmc->checkReservedInPort(lHandMomentSensorIn())) {
      std::cout << "ERROR!!!: "
                << lHandMomentSensorIn()->getPortName() << " (or "
                << lHandWrenchSensorIn()->getPortName() << ") is not connected in "
                << getTaskName() << std::endl;
      return false;
    }    
  }
  
  if (!hmc->checkReservedInPort(tauIn())) {
    std::cout << "ERROR!!!: " << tauIn()->getPortName() << " is not connected in "
              << getTaskName() << std::endl;
    return false;
  }
  
  return TaskExecutionManager::onActivate();
}

bool MultiContactMotionSolver::onExecute(void)
{
  // std::cout << "Rafa, in MCMS::onExecute, *m_qIn = " << (*m_qIn).transpose() << std::endl;

  dvector q_in = dvector::Zero(m_body->numJoints());
  
  if (qIn()->isConnected() && !containsNan(*m_qIn))
      q_in = *m_qIn;

  robot().encoderValues(eigen2std(q_in));
  m_body->setPosture(q_in);

  Vector3 waistP_in = Vector3::Zero();
  
  if (waistPIn()->isConnected() && !containsNan(*m_waistPIn))
    waistP_in = *m_waistPIn;
  else if (waistPoseIn()->isConnected() && !containsNan(m_waistPoseIn->P))
    waistP_in = m_waistPoseIn->P;
  else if (waistStateIn()->isConnected() && !containsNan(m_waistStateIn->P))
    waistP_in = m_waistStateIn->P;

  robot().bodySensor().position(waistP_in);

  Matrix33 waistR_in = Matrix33::Identity();
  
  if (waistRIn()->isConnected() && !containsNan(*m_waistRIn))
    waistR_in = *m_waistRIn;
  else if (waistPoseIn()->isConnected() && !containsNan(m_waistPoseIn->R))
    waistR_in = m_waistPoseIn->R;
  else if (waistStateIn()->isConnected() && !containsNan(m_waistStateIn->R))
    waistR_in = m_waistStateIn->R;
  
  robot().bodySensor().orientation(Eigen::Quaterniond(waistR_in));
  
  m_body->rootLink()->p = waistP_in;
  m_body->rootLink()->R = waistR_in;

  // std::cout << "Rafa, in MCMS::onExecute, before solving, m_body->rootLink()->p = " << m_body->rootLink()->p.transpose() << std::endl;
  // std::cout << "Rafa, in MCMS::onExecute, m_body->rootLink()->R = " << std::endl << m_body->rootLink()->R << std::endl;

  Vector3 waistLinVel_in = Vector3::Zero();
  
  if (waistLinVelIn()->isConnected() && !containsNan(*m_waistLinVelIn))
    waistLinVel_in = *m_waistLinVelIn;
  else if (waistGeneralizedVelIn()->isConnected() && !containsNan(*m_waistGeneralizedVelIn))
    waistLinVel_in = m_waistGeneralizedVelIn->head(3);
  else if (waistStateIn()->isConnected() && !containsNan(m_waistStateIn->V))
    waistLinVel_in = m_waistStateIn->V;
  
  robot().bodySensor().linearVelocity(waistLinVel_in);

  Vector3 waistAngVel_in = Vector3::Zero();
  
  if (waistAngVelIn()->isConnected() && !containsNan(*m_waistAngVelIn))
    waistAngVel_in = *m_waistAngVelIn;
  else if (waistGeneralizedVelIn()->isConnected() && !containsNan(*m_waistGeneralizedVelIn))
    waistAngVel_in = m_waistGeneralizedVelIn->tail(3);
  else if (waistStateIn()->isConnected() && !containsNan(m_waistStateIn->W))
    waistAngVel_in = m_waistStateIn->W;
  
  robot().bodySensor().angularVelocity(waistAngVel_in);

  Vector3 waistLinAcc_in = Vector3::Zero();
  
  if (waistLinAccIn()->isConnected() && !containsNan(*m_waistLinAccIn))
    waistLinAcc_in = *m_waistLinAccIn;
  else if (waistStateIn()->isConnected() && !containsNan(m_waistStateIn->Vdot))
    waistLinAcc_in = m_waistStateIn->Vdot;

  robot().bodySensor().acceleration(waistLinAcc_in);

  Vector3 rFootForceSensor_in = Vector3::Zero();
  
  if (rFootForceSensorIn()->isConnected() && !containsNan(*m_rFootForceSensorIn))
    rFootForceSensor_in = *m_rFootForceSensorIn;
  else if (rFootWrenchSensorIn()->isConnected() && !containsNan(*m_rFootWrenchSensorIn))
    rFootForceSensor_in = m_rFootWrenchSensorIn->head(3);

  m_wrenches["RightFootForceSensor"].force() = rFootForceSensor_in;

  Vector3 rFootMomentSensor_in = Vector3::Zero();
  
  if (rFootMomentSensorIn()->isConnected() && !containsNan(*m_rFootMomentSensorIn))
    rFootMomentSensor_in = *m_rFootMomentSensorIn;
  else if (rFootWrenchSensorIn()->isConnected() && !containsNan(*m_rFootWrenchSensorIn))
    rFootMomentSensor_in = m_rFootWrenchSensorIn->tail(3);

  m_wrenches["RightFootForceSensor"].couple() = rFootMomentSensor_in;

  Vector3 lFootForceSensor_in = Vector3::Zero();
  
  if (lFootForceSensorIn()->isConnected() && !containsNan(*m_lFootForceSensorIn))
    lFootForceSensor_in = *m_lFootForceSensorIn;
  else if (lFootWrenchSensorIn()->isConnected() && !containsNan(*m_lFootWrenchSensorIn))
    lFootForceSensor_in = m_lFootWrenchSensorIn->head(3);

  m_wrenches["LeftFootForceSensor"].force() = lFootForceSensor_in;

  Vector3 lFootMomentSensor_in = Vector3::Zero();
  
  if (lFootMomentSensorIn()->isConnected() && !containsNan(*m_lFootMomentSensorIn))
    lFootMomentSensor_in = *m_lFootMomentSensorIn;
  else if (lFootWrenchSensorIn()->isConnected() && !containsNan(*m_lFootWrenchSensorIn))
    lFootMomentSensor_in = m_lFootWrenchSensorIn->tail(3);

  m_wrenches["LeftFootForceSensor"].couple() = lFootMomentSensor_in;

  Vector3 rHandForceSensor_in = Vector3::Zero();
  
  if (rHandForceSensorIn()->isConnected() && !containsNan(*m_rHandForceSensorIn))
    rHandForceSensor_in = *m_rHandForceSensorIn;
  else if (rHandWrenchSensorIn()->isConnected() && !containsNan(*m_rHandWrenchSensorIn))
    rHandForceSensor_in = m_rHandWrenchSensorIn->head(3);

  m_wrenches["RightHandForceSensor"].force() = rHandForceSensor_in;

  Vector3 rHandMomentSensor_in = Vector3::Zero();
  
  if (rHandMomentSensorIn()->isConnected() && !containsNan(*m_rHandMomentSensorIn))
    rHandMomentSensor_in = *m_rHandMomentSensorIn;
  else if (rHandWrenchSensorIn()->isConnected() && !containsNan(*m_rHandWrenchSensorIn))
    rHandMomentSensor_in = m_rHandWrenchSensorIn->tail(3);

  m_wrenches["RightHandForceSensor"].couple() = rHandMomentSensor_in;

  Vector3 lHandForceSensor_in = Vector3::Zero();
  
  if (lHandForceSensorIn()->isConnected() && !containsNan(*m_lHandForceSensorIn))
    lHandForceSensor_in = *m_lHandForceSensorIn;
  else if (lHandWrenchSensorIn()->isConnected() && !containsNan(*m_lHandWrenchSensorIn))
    lHandForceSensor_in = m_lHandWrenchSensorIn->head(3);

  m_wrenches["LeftHandForceSensor"].force() = lHandForceSensor_in;

  Vector3 lHandMomentSensor_in = Vector3::Zero();
  
  if (lHandMomentSensorIn()->isConnected() && !containsNan(*m_lHandMomentSensorIn))
    lHandMomentSensor_in = *m_lHandMomentSensorIn;
  else if (lHandWrenchSensorIn()->isConnected() && !containsNan(*m_lHandWrenchSensorIn))
    lHandMomentSensor_in = m_lHandWrenchSensorIn->tail(3);

  m_wrenches["LeftHandForceSensor"].couple() = lHandMomentSensor_in;
  
  for (const std::pair<const std::string, sva::ForceVecd> & w : m_wrenches)
    robot().forceSensor(w.first).wrench(w.second);

  dvector tau_in = dvector::Zero(m_body->numJoints());
  
  if (tauIn()->isConnected() && !containsNan(*m_tauIn))
    tau_in = *m_tauIn;
    
  robot().jointTorques(eigen2std(tau_in));

  Vector3  total_com;
  Matrix33 total_inr;

  m_body->calcForwardKinematics();
  m_body->calcComInertia(total_com, total_inr, m_linkInr);

  if (m_running) {
    
    double t = 0.0;  // Currently unused, arbitrary

    clock_t time;

    if (!m_init) {
      
      LOG_INFO("Init controller");

      // std::cout << "Rafa, in MultiContactMotionSolver::onExecute, *m_qInit = " << (*m_qInit).transpose() << std::endl;

      if (qInit()->isConnected() && !containsNan(*m_qInit)) {
        if (waistPoseInit()->isConnected() && !containsNan(m_waistPoseInit->P) && !containsNan(m_waistPoseInit->R)) {
          Eigen::Quaterniond waistQt(m_waistPoseInit->R);
          std::array<double, 7> waistPose = {waistQt.w(), waistQt.x(), waistQt.y(), waistQt.z(),
                                             m_waistPoseInit->P.x(), m_waistPoseInit->P.y(), m_waistPoseInit->P.z()};
          init(eigen2std(*m_qInit), waistPose);
        }
        else
          init(eigen2std(*m_qInit));
      }
      
      m_init = true;
    }

    time = clock();

    if (m_init && run()) {

      TorqueConstraint* torqueConstraint = hmc->findTaskExecution<TorqueConstraint*>();
      if (torqueConstraint)
        solver().fillTorque(torqueConstraint->torque_constr());

      // std::cout << "Rafa, in MCMS::onExecute, after solving, m_body->rootLink()->p = " << m_body->rootLink()->p.transpose() << std::endl;
      
      executeTasks();

      const mc_solver::QPResultMsg & res = m_solver->send(t);

      m_waistStateRefOut.P = Vector3(robot().mbc().q[0][4], robot().mbc().q[0][5], robot().mbc().q[0][6]);
      Eigen::Quaterniond waistQt(robot().mbc().q[0][0], robot().mbc().q[0][1], robot().mbc().q[0][2], robot().mbc().q[0][3]);
      m_waistStateRefOut.R = waistQt.normalized().toRotationMatrix();

      // std::cout << "Rafa, in MultiContactMotionSolver::onExecute, m_waistStateRefOut.P = " << m_waistStateRefOut.P.transpose() << std::endl;

      if (qOmitRefIn()->isConnected() && !containsNan(*m_qOmitRefIn))
        m_qRefOut = *m_qOmitRefIn;
      
      const std::vector<std::string> & ref_joint_order = robot().refJointOrder();

      for (size_t i = 0; i < ref_joint_order.size(); i++) {

        if (robot().hasJoint(ref_joint_order[i])) {

          int jointIndex = robot().mb().jointIndexByName(ref_joint_order[i]);
          
          if (robot().mb().joint(jointIndex).dof() == 0)  // Rafa changed this for OmitJoints
            continue;
          
          m_qRefOut[i]   = res.robots_state[0].q.at(ref_joint_order[i])[0];
          m_dqRefOut[i]  = res.robots_state[0].alphaVec.at(ref_joint_order[i])[0];
          m_ddqRefOut[i] = res.robots_state[0].alphaDVec.at(ref_joint_order[i])[0];
          m_tauRefOut[i] = res.robots_state[0].torque.at(ref_joint_order[i])[0];
        }
      }

      // std::cout << "Rafa, in MultiContactMotionSolver::onExecute, m_qRefOut[16] = " << m_qRefOut[16] << std::endl;

      // std::cout << "Rafa, in MCMS::onExecute, m_waistStateRefOut.P = " << m_waistStateRefOut.P.transpose() << std::endl;
      // std::cout << "Rafa, in MCMS::onExecute, *m_qIn = " << (*m_qIn).transpose() << std::endl;
      // std::cout << "Rafa, in MCMS::onExecute, m_qRefOut = " << m_qRefOut.transpose() << std::endl;
      // std::cout << "Rafa, in MCMS::onExecute, m_tauRefOut = " << m_tauRefOut.transpose() << std::endl;

      if (fbTerm()) {
        m_tauPOut = fbTerm()->P();
        m_gammaDOut = fbTerm()->gammaD();
      }

      // Rafa, why does this patch need to be applied? Check later
      if (res.lambdaVec.size()) {

        // bool nantest = false;

        // for (int i = 0; i < res.lambdaVec.size(); i++)
        // nantest |= std::isnan(res.lambdaVec(i));
        
        // if (!nantest)
        publish_reference_wrenches(res);
        // else
        // std::cout << "Rafa, it is NaN!" << std::endl;
      }

      publish_measured_wrenches();
    }
    else {
      m_running = false;
      m_init = false;
      m_failed = true;

      enable_feedback_torque_mode(false);

      if (waistPIn()->isConnected() && !containsNan(*m_waistPIn))
        m_waistStateRefOut.P = *m_waistPIn;
      else if (waistPoseIn()->isConnected() && !containsNan(m_waistPoseIn->P))
        m_waistStateRefOut.P = m_waistPoseIn->P;
      else if (waistStateIn()->isConnected() && !containsNan(m_waistStateIn->P))
        m_waistStateRefOut.P = m_waistStateIn->P;

      if (waistRIn()->isConnected() && !containsNan(*m_waistRIn))
        m_waistStateRefOut.R = *m_waistRIn;
      else if (waistPoseIn()->isConnected() && !containsNan(m_waistPoseIn->R))
        m_waistStateRefOut.R = m_waistPoseIn->R;
      else if (waistStateIn()->isConnected() && !containsNan(m_waistStateIn->R))
        m_waistStateRefOut.R = m_waistStateIn->R;

      if (qIn()->isConnected() && !containsNan(*m_qIn))
        m_qRefOut = *m_qIn;
    }

    time = clock() - time;

    std::cout << "Rafa, the time spent is " << time << std::endl;
  }
  else {

    if (!m_failed) {
      if (waistPIn()->isConnected() && !containsNan(*m_waistPIn))
        m_waistStateRefOut.P = *m_waistPIn;
      else if (waistPoseIn()->isConnected() && !containsNan(m_waistPoseIn->P))
        m_waistStateRefOut.P = m_waistPoseIn->P;
      else if (waistStateIn()->isConnected() && !containsNan(m_waistStateIn->P))
        m_waistStateRefOut.P = m_waistStateIn->P;

      if (waistRIn()->isConnected() && !containsNan(*m_waistRIn))
        m_waistStateRefOut.R = *m_waistRIn;
      else if (waistPoseIn()->isConnected() && !containsNan(m_waistPoseIn->R))
        m_waistStateRefOut.R = m_waistPoseIn->R;
      else if (waistStateIn()->isConnected() && !containsNan(m_waistStateIn->R))
        m_waistStateRefOut.R = m_waistStateIn->R;

      if (qIn()->isConnected() && !containsNan(*m_qIn))
        m_qRefOut = *m_qIn;
    }
  }

  // std::cout << "Rafa, in MCMS::onExecute, m_waistRefOut.P = " << m_waistRefOut.P.transpose() << std::endl;
  // std::cout << "Rafa, in MCMS::onExecute, *m_qIn = " << m_qIn->transpose() << std::endl;
  // std::cout << "Rafa, in MCMS::onExecute, m_qRefOut = " << m_qRefOut.transpose() << std::endl << std::endl;
  
  return true;
}

const motion_generator::HumanoidBodyPtr MultiContactMotionSolver::body()
{
  return m_body;
}

const std::vector<motion_generator::LinkInr> & MultiContactMotionSolver::linkInr()
{
  return m_linkInr;
}

mc_rbdyn::Robot  & MultiContactMotionSolver::robot()
{
  return m_solver->robot();
}

mc_rbdyn::Robots & MultiContactMotionSolver::robots()
{
  return m_solver->robots();
}

mc_rbdyn::RobotModule & MultiContactMotionSolver::robot_module()
{
  return *m_robot_module;
}

const torque_control::TorqueFeedbackTerm::TorqueControlType MultiContactMotionSolver::torqueControlType()
{
  return m_torqueControlType;
}

const std::shared_ptr<torque_control::TorqueFeedbackTerm> MultiContactMotionSolver::fbTerm() const
{
  mc_solver::IntglTerm_QPSolver* intglTerm_solver;
  mc_solver::PassivityPIDTerm_QPSolver* passivityPIDTerm_solver;

  intglTerm_solver = dynamic_cast<mc_solver::IntglTerm_QPSolver*>(m_solver); 
  passivityPIDTerm_solver = dynamic_cast<mc_solver::PassivityPIDTerm_QPSolver*>(m_solver);
  
  if (intglTerm_solver)
    return intglTerm_solver->fbTerm();
  else if (passivityPIDTerm_solver)
    return passivityPIDTerm_solver->fbTerm();
  else
    return NULL;
}

mc_solver::QPSolver & MultiContactMotionSolver::solver() {
  return *m_solver;
}

void MultiContactMotionSolver::addTask(tasks::qp::Task* task)
{
  std::lock_guard<std::mutex> l(m_mutex);
  m_solver->addTask(task);
}

void MultiContactMotionSolver::addTask(mc_tasks::MetaTask* task)
{
  std::lock_guard<std::mutex> l(m_mutex);  
  m_solver->addTask(task);
}

void MultiContactMotionSolver::removeTask(tasks::qp::Task* task)
{
  std::lock_guard<std::mutex> l(m_mutex);
  m_solver->removeTask(task);
}
  
void MultiContactMotionSolver::removeTask(mc_tasks::MetaTask* task)
{
  std::lock_guard<std::mutex> l(m_mutex);
  m_solver->removeTask(task);
}

void MultiContactMotionSolver::addConstraint(tasks::qp::ConstraintFunction<tasks::qp::Equality>* constraint)
{
  std::lock_guard<std::mutex> l(m_mutex);
  m_solver->addConstraint(constraint);
  m_solver->updateConstrSize();
}

void MultiContactMotionSolver::addConstraint(tasks::qp::ConstraintFunction<tasks::qp::Inequality>* constraint)
{
  std::lock_guard<std::mutex> l(m_mutex);
  m_solver->addConstraint(constraint);
  m_solver->updateConstrSize();
}

void MultiContactMotionSolver::addConstraint(tasks::qp::ConstraintFunction<tasks::qp::GenInequality>* constraint)
{
  std::lock_guard<std::mutex> l(m_mutex);
  m_solver->addConstraint(constraint);
  m_solver->updateConstrSize();
}

void MultiContactMotionSolver::addConstraint(tasks::qp::ConstraintFunction<tasks::qp::Bound>* constraint)
{
  std::lock_guard<std::mutex> l(m_mutex);
  m_solver->addConstraint(constraint);
  m_solver->updateConstrSize();
}

void MultiContactMotionSolver::addConstraintSet(mc_solver::ConstraintSet & constraint_set)
{
  std::lock_guard<std::mutex> l(m_mutex);
  m_solver->addConstraintSet(constraint_set);
}

void MultiContactMotionSolver::removeConstraint(tasks::qp::ConstraintFunction<tasks::qp::Equality>* constraint)
{
  std::lock_guard<std::mutex> l(m_mutex);
  m_solver->removeConstraint(constraint);
  m_solver->updateConstrSize();
}

void MultiContactMotionSolver::removeConstraint(tasks::qp::ConstraintFunction<tasks::qp::Inequality>* constraint)
{
  std::lock_guard<std::mutex> l(m_mutex);
  m_solver->removeConstraint(constraint);
  m_solver->updateConstrSize();
}

void MultiContactMotionSolver::removeConstraint(tasks::qp::ConstraintFunction<tasks::qp::GenInequality>* constraint)
{
  std::lock_guard<std::mutex> l(m_mutex);
  m_solver->removeConstraint(constraint);
  m_solver->updateConstrSize();
}

void MultiContactMotionSolver::removeConstraint(tasks::qp::ConstraintFunction<tasks::qp::Bound>* constraint)
{
  std::lock_guard<std::mutex> l(m_mutex);
  m_solver->removeConstraint(constraint);
  m_solver->updateConstrSize();
}

void MultiContactMotionSolver::removeConstraintSet(mc_solver::ConstraintSet & constraint_set)
{
  std::lock_guard<std::mutex> l(m_mutex);
  m_solver->removeConstraintSet(constraint_set);
}

void MultiContactMotionSolver::setContacts(const std::vector<mc_rbdyn::Contact> & contacts)
{
  std::lock_guard<std::mutex> l(m_mutex);
  m_solver->setContacts(contacts);
  
  std::cout << "Current contact set:" << std::endl;

  for (const auto & c : m_solver->data().allContacts()) {
    const auto & cId = c.contactId;
    std::cout << "- " << cId.r1BodyName << "/" << cId.r2BodyName << " (" << cId.r1Index << "/" << cId.r2Index << ")" << std::endl;
  }
}

bool MultiContactMotionSolver::init(const std::vector<double> & initq)
{
  init(initq, m_robot_module->default_attitude());
}

bool MultiContactMotionSolver::init(const std::vector<double> & initq, const std::array<double, 7> & initAttitude)
{
  // std::cout << "Rafa, in MultiContactMotionsolver::init, initq[3] = " << initq[3] << std::endl;
  
  std::vector<std::vector<double>> q = robot().mbc().q;
  q[0] = {std::begin(initAttitude), std::end(initAttitude)};
  
  const std::vector<std::string> & ref_joint_order = robot().refJointOrder();
  
  for (size_t i = 0; i < ref_joint_order.size(); i++) {

    const std::string & joint_name = ref_joint_order[i];

    if (robot().hasJoint(joint_name))
    {
      auto jIndex = robot().jointIndexByName(joint_name);

      // std::cout << "Rafa, in MultiContactMotionsolver::init, joint_name = " << joint_name << ", jIndex = " << jIndex << ", q[jIndex].size() = " << q[jIndex].size() << std::endl;

      if (q[jIndex].size() == 0)
        continue;

      q[jIndex][0] = initq[i];
      // std::cout << "Rafa, in MultiContactMotionsolver::init, q[" << jIndex << "][0] = " << q[jIndex][0] << std::endl;
      // std::cout << "Rafa, in MultiContactMotionsolver::init, robot().mbc().q[" << jIndex << "][0] = " << robot().mbc().q[jIndex][0] << std::endl;
    }
  }

  // std::cout << "Rafa, in MultiContactMotionsolver::init, robot().mbc().q[4][0] = " << robot().mbc().q[4][0] << std::endl;
  
  reset({q});
}

bool MultiContactMotionSolver::run()
{
  std::lock_guard<std::mutex> l(m_mutex);
  if (m_running && m_solver->run(true))
    return true;
  else {
    LOG_ERROR("QP failed to run()");
    return false;
  }
}

void MultiContactMotionSolver::reset(const std::vector<std::vector<double>> & q)
{
  robot().mbc().zero(robot().mb());
  robot().mbc().q = q;

  // std::cout << "Rafa, in MultiContactMotionsolver::reset, robot().mbc().q[4][0] = " << robot().mbc().q[4][0] << std::endl;
  
  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());

  for (TaskExecutionHandler* handler : Tasks())
    if (handler->isActive()) {
      Task* task = dynamic_cast<Task*>(handler);
      if (task != NULL)
        task->reset();
    }
}

void MultiContactMotionSolver::addRotorInertia()
{
  addRotorInertia(robot());
}

void MultiContactMotionSolver::addRotorInertia(mc_rbdyn::Robot& robot)
{
  const std::vector<std::string> & ref_joint_order = robot.refJointOrder();
  
  for (size_t i = 0; i < ref_joint_order.size(); i++) {

    const std::string & joint_name = ref_joint_order[i];

    if (robot.hasJoint(joint_name)) {

      int j = robot.jointIndexByName(joint_name);

      double Ir = m_body->joint(i)->Ir;
      double gr = m_body->joint(i)->gearRatio;
      
      robot.mb().setJointRotorInertia(j, Ir);
      robot.mb().setJointGearRatio(j, gr);
    }
  }

  robot.fd()->computeHIr(robot.mb());
}

Eigen::Vector6d MultiContactMotionSolver::computeWrench(std::string linkName, const tasks::qp::BilateralContact & contact, Eigen::VectorXd lambdaVec, unsigned int pos)
{
  dvector6 wrench = dvector6::Zero();

  Vector3 force  = Vector3::Zero();
  Vector3 moment = Vector3::Zero();

  // std::cout << "Rafa, in MultiContactMotionSolver::computeWrench, linkName = " << linkName << std::endl;
  // std::cout << "Rafa, in MultiContactMotionSolver::computeWrench, robot().mbc().bodyPosW[linkIndex].rotation() = " << std::endl << robot().mbc().bodyPosW[robot().bodyIndexByName(linkName)].rotation() << std::endl;
  
  for (size_t i = 0; i < contact.r1Points.size(); i++) {

    dvector lambda = Eigen::VectorXd::Zero(contact.nrLambda(i));
    
    if (lambdaVec.size() > 0 && pos < lambdaVec.size())
    {
      lambda = lambdaVec.segment(pos, contact.nrLambda(i));
    }
    
    int linkIndex = robot().bodyIndexByName(linkName);

    // std::cout << "Rafa, in MultiContactMotionSolver::computeWrench, lambdaVec.size() = " << lambdaVec.size() << std::endl;
    
    // std::cout << "Rafa, in MultiContactMotionSolver::computeWrench, for i = " << i << " and pos = " << pos << ", lambda = " << lambda.transpose() << std::endl;
    
    force  += contact.force(lambda, i, contact.r1Cones);
    // moment += (robot().mbc().bodyPosW[linkIndex].rotation() * contact.r1Points[i]).cross(contact.force(lambda, i, contact.r1Cones));
    moment += (robot().mbc().bodyPosW[linkIndex].rotation().transpose() * contact.r1Points[i]).cross(contact.force(lambda, i, contact.r1Cones));

    // std::cout << "Rafa, in MultiContactMotionSolver::computeWrench, contact.r1Points[" << i << "] = " << contact.r1Points[i].transpose() << std::endl;
    // std::cout << "Rafa, in MultiContactMotionSolver::computeWrench, robot().mbc().bodyPosW[linkIndex].rotation().transpose() * contact.r1Points[" << i << "] = " << (robot().mbc().bodyPosW[linkIndex].rotation().transpose() * contact.r1Points[i]).transpose() << std::endl;
    // std::cout << "Rafa, in MultiContactMotionSolver::computeWrench, contact.force(lambda, i, contact.r1Cones) = " << contact.force(lambda, i, contact.r1Cones).transpose() << std::endl;
    
    pos += contact.nrLambda(i);
  }

  wrench.head(3) = force;
  wrench.tail(3) = moment;
  
  return wrench;
}

void MultiContactMotionSolver::publish_reference_wrenches(const mc_solver::QPResultMsg & res)
{
  std::map<std::string, Eigen::Vector6d> wrenches;
  
  for (size_t ci = 0; ci < res.contacts.size(); ci++) {
    
    const mc_solver::ContactMsg & c = res.contacts[ci];
    
    if (c.r1_index == 0) {
      
      mc_rbdyn::Contact con(robots(), 0, c.r2_index, c.r1_surface, c.r2_surface);
      std::pair<int, const tasks::qp::BilateralContact&> indexed_contact = solver().contactById(con.contactId(robots()));
      
      if (indexed_contact.first != -1) {
        
        const tasks::qp::BilateralContact & contact = indexed_contact.second;
        int pos = res.contacts_lambda_begin[ci];

        // std::cout << "Rafa, in MultiContactMotionSolver::publish_reference_wrenches, res.contacts_lambda_begin[ci] = " << pos << std::endl;

        // std::cout << "Rafa, in MultiContactMotionSolver::publish_reference_wrenches, c.r1_body = " << c.r1_body << std::endl;

        // std::cout << "Rafa, in MultiContactMotionSolver::publish_reference_wrenches, res.lambdaVec = " << res.lambdaVec.transpose() << std::endl;


        // Rafa added this
        // bool nantest = false;
        // for (int i = 0; i < res.lambdaVec.size(); i++) {
        //   nantest |= std::isnan(res.lambdaVec(i));
        // }
        // if (nantest)
        // std::cout << "Rafa, in MultiContactMotionSolver::publish_reference_wrenches, res.lambdaVec = " << res.lambdaVec.transpose() << std::endl;          
        
        wrenches[c.r1_body] = computeWrench(c.r1_body, contact, res.lambdaVec, pos);

        // std::cout << "Rafa, in MultiContactMotionSolver::publish_reference_wrenches, wrenches[c.r1_body].head(3) = " << wrenches[c.r1_body].head(3).transpose() << std::endl << std::endl;
      }
    }
  }

  std::vector<std::string> EEJointName = {m_body->ankleLink[0]->name,
                                          m_body->ankleLink[1]->name,
                                          m_body->wristLink[0]->name,
                                          m_body->wristLink[1]->name};

  std::vector<dvector6*> EEWrenchRefOut = {&m_rFootWrenchRefOut,
                                           &m_lFootWrenchRefOut,
                                           &m_rHandWrenchRefOut,
                                           &m_lHandWrenchRefOut};
  
  for (size_t i = 0; i < EEJointName.size(); i++) {

    int EEJointId = robot().mb().jointIndexByName().at(EEJointName[i]);
    int EELinkId = robot().mb().successor(EEJointId);
    std::string EELinkName = robot().mb().body(EELinkId).name();
    
    std::map<std::string, Eigen::Vector6d>::iterator it;

    it = wrenches.find(EELinkName);
    if (it != wrenches.end())
      *EEWrenchRefOut[i] = it->second;
    else
      *EEWrenchRefOut[i] = dvector6::Zero();
  }
}

void MultiContactMotionSolver::publish_measured_wrenches()
{
  std::vector<std::string> EESensorName = {"RightFootForceSensor",
                                           "LeftFootForceSensor",
                                           "RightHandForceSensor",
                                           "LeftHandForceSensor"};

  std::vector<std::string> EEJointName = {m_body->ankleLink[0]->name,
                                          m_body->ankleLink[1]->name,
                                          m_body->wristLink[0]->name,
                                          m_body->wristLink[1]->name};

  std::vector<dvector6*> EEWrenchSensorIn = {m_rFootWrenchSensorIn,
                                             m_lFootWrenchSensorIn,
                                             m_rHandWrenchSensorIn,
                                             m_lHandWrenchSensorIn};

  std::vector<Vector3*> EEForceSensorIn = {m_rFootForceSensorIn,
                                           m_lFootForceSensorIn,
                                           m_rHandForceSensorIn,
                                           m_lHandForceSensorIn};  

  std::vector<Vector3*> EEMomentSensorIn = {m_rFootMomentSensorIn,
                                            m_lFootMomentSensorIn,
                                            m_rHandMomentSensorIn,
                                            m_lHandMomentSensorIn}; 
  
  std::vector<dvector6*> EEWrenchHatOut = {&m_rFootWrenchHatOut,
                                           &m_lFootWrenchHatOut,
                                           &m_rHandWrenchHatOut,
                                           &m_lHandWrenchHatOut};

  for (size_t i = 0; i < EESensorName.size(); i++) {

    int EEJointId = robot().mb().jointIndexByName().at(EEJointName[i]);
    int EELinkId = robot().mb().successor(EEJointId);
    
    std::string SensorLinkName = robot().forceSensor(EESensorName[i]).parentBody();
    int SensorLinkId = robot().mb().bodyIndexByName(SensorLinkName);
    std::string SensorJointName = robot().mb().joint(SensorLinkId).name();
    
    int      index = m_body->link(SensorJointName)->index;
    double   EEMass = m_linkInr[index].Mass;
    Vector3  EECom = m_linkInr[index].Com;
    Matrix33 EEInr = m_linkInr[index].Inr;
    
    sva::PTransformd SensorRelT = robot().forceSensor(EESensorName[i]).X_p_f();
    // Matrix33 SensorLinkRot = robot().mbc().bodyPosW[SensorLinkId].rotation();
    Matrix33 SensorLinkRot = robot().mbc().bodyPosW[SensorLinkId].rotation().transpose();
    // Matrix33 SensorRot = SensorLinkRot * SensorRelT.rotation();
    Matrix33 SensorRot = SensorLinkRot * SensorRelT.rotation().transpose();
    Vector3  SensorPos = robot().mbc().bodyPosW[SensorLinkId].translation() + SensorLinkRot * SensorRelT.translation();

    rbd::Jacobian jac(robot().mb(), SensorLinkName, EECom);
    dmatrix jacMat(6, robot().mb().nrDof()), jacDotMat(6, robot().mb().nrDof());
    jac.fullJacobian(robot().mb(), jac.jacobian(robot().mb(), robot().mbc()), jacMat);
    jac.fullJacobian(robot().mb(), jac.jacobianDot(robot().mb(), robot().mbc()), jacDotMat);

    dvector alphaVec_hat = rbd::dofToVector(robot().mb(), robot().mbc().alpha);
    dvector alphaDotVec_hat;
    if (!m_first)
      alphaDotVec_hat = (alphaVec_hat - m_alphaVec_hat_prev) / m_dt;
    else {
      alphaDotVec_hat = dvector::Zero(robot().mb().nrDof());
      m_first = false;
    }
    m_alphaVec_hat_prev = alphaVec_hat;

    dvector  alphaDotVec_ref = rbd::dofToVector(robot().mb(), robot().mbc().alphaD);
    dvector  alphaDotVec_fil = m_CompFilter.CF(alphaDotVec_hat, alphaDotVec_ref);
    
    dvector6 EEAcc = jacMat * alphaDotVec_fil + jacDotMat * alphaVec_hat;

    Vector3 EELinkPos = robot().mbc().bodyPosW[EELinkId].translation();
    Vector3 EEAngVel  = robot().mbc().bodyVelW[SensorLinkId].angular();

    Vector3 EEWeight    = {0.0, 0.0, EEMass * -9.81};
    Vector3 EEInrForce  = -(EEMass * EEAcc.tail(3));
    Vector3 EEInrMoment = -(EEInr * EEAcc.head(3) + EEAngVel.cross(EEInr * EEAngVel));

    Vector3 EEForceSensorW = Vector3::Zero();
    Vector3 EEMomentSensorW = Vector3::Zero();

    if (EEForceSensorIn[i])
      EEForceSensorW = SensorRot * (*EEForceSensorIn[i]);
    else if (EEWrenchSensorIn[i])
      EEForceSensorW = SensorRot * EEWrenchSensorIn[i]->head(3);

    if (EEMomentSensorIn[i])
      EEMomentSensorW = SensorRot * (*EEMomentSensorIn[i]);
    else if (EEWrenchSensorIn[i])
      EEMomentSensorW = SensorRot * EEWrenchSensorIn[i]->tail(3);
        
    EEWrenchHatOut[i]->head(3) = EEForceSensorW - EEWeight; // - EEInrForce;
    EEWrenchHatOut[i]->tail(3) = EEMomentSensorW - (SensorPos - EELinkPos).cross(EEForceSensorW) - (EECom - EELinkPos).cross(EEWeight + EEInrForce); // - EEInrMoment;
  }
}

void MultiContactMotionSolver::enable_feedback_torque_mode(bool flag)
{
  m_solver->enableFeedback(flag);

  m_torqueModeOut = std::vector<bool>(m_body->numJoints(), flag);
  
  if (flag) {
    
    for (std::string & joint : m_omitted_joints)
      m_torqueModeOut[m_body->link(joint)->jointId] = false;

    std::vector<LinkTraverse*> wrist(2);

    for (size_t i = 0; i < wrist.size(); i++) {
      wrist[i] = new LinkTraverse(m_body->wristLink[i]);
      for (size_t j = 1; j < wrist[i]->size(); j++)
        m_torqueModeOut[wrist[i]->link(j)->jointId] = false;
    }
  }
}

bool MultiContactMotionSolver::
cmd_enable_solver(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  bool i_flag = false;
  i_strm >> i_flag;

  if (i_flag) {
    m_running = true;
    o_strm << "enabled controller" << std::endl;
  }
  else {
    m_running = false;
    o_strm << "disabled controller" << std::endl;
  }

  return true;
}

bool MultiContactMotionSolver::
cmd_omit_joints_in_solver(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  if (!m_solver && m_omitted_joints.size() == 0) {

    while (i_strm.good()) {

      std::string joint;
      i_strm >> joint;

      if (m_body->link(joint))
        m_omitted_joints.push_back(joint);
    }

    if (m_omitted_joints.size() == 0)
      o_strm << "no joints have been specified to be omitted" << std::endl;
    else {
      o_strm << "omitting the following joints in the solver: ";
      for (size_t i = 0; i < m_omitted_joints.size(); i++)
        o_strm << m_omitted_joints[i] << " ";
      o_strm << std::endl;
    }

    return true;
  }
  else {
    
    o_strm << "WARNING! cannot specify ommited joints after the Robot has been registered" << std::endl;
    
    return false;
  }
}

bool MultiContactMotionSolver::
cmd_enable_feedback_torque_mode(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  bool i_flag = false;
  i_strm >> i_flag;

  enable_feedback_torque_mode(i_flag);

  if (i_flag)
    o_strm << "enabled feedback/torque mode in non-omitted joints" << std::endl;
  else
    o_strm << "disabled feedback/torque mode in non-omitted joints" << std::endl;

  o_strm << "torque mode vector: ";
  for (size_t i = 0; i < m_torqueModeOut.size(); i++)
    o_strm << m_torqueModeOut[i] << " ";
  o_strm << std::endl;
  
  return true;
}

bool MultiContactMotionSolver::
cmd_select_torque_control_type(std::istringstream& i_strm, std::ostringstream& o_strm)
{
  if (!m_solver) {

    std::string torqueControlType;
    i_strm >> torqueControlType;

    if (torqueControlType == "IntegralTerm") {

      m_torqueControlType = torque_control::TorqueFeedbackTerm::IntegralTerm;
      o_strm << "selecting the integral term type of feedback control ";
    
      std::string intglTermType;
      i_strm >> intglTermType;

      if (intglTermType == "PassivityBased") {
        m_intglTermType = torque_control::IntegralTerm::PassivityBased;
        o_strm << "with a passivity-based integral term (L = C + K), ";
      }
      else if (intglTermType == "Simple") {
        m_intglTermType = torque_control::IntegralTerm::Simple;
        o_strm << "with a simple integral term (L = K), ";
      }
      else {
        m_intglTermType = torque_control::IntegralTerm::None;
        o_strm << "without using any integral term (same as no torque control term)" << std::endl;
      }

      if (m_intglTermType != torque_control::IntegralTerm::None) {

        i_strm >> m_lambda;

        std::string velGainType;
        i_strm >> velGainType;
          
        if (velGainType == "MassMatrix") {
          m_velGainType = torque_control::IntegralTerm::MassMatrix;
          o_strm << "by using velocity gain: K = lambda * M, with lambda = " << m_lambda << std::endl;
        }
        else if (velGainType == "MassDiagonal") {
          m_velGainType = torque_control::IntegralTerm::MassDiagonal;
          o_strm << "by using velocity gain: K = lambda * diag(M), with lambda = " << m_lambda << std::endl;
        }
        else {
          m_velGainType = torque_control::IntegralTerm::Diagonal;
          o_strm << "by using velocity gain: K = lambda * eye, with lambda = " << m_lambda << std::endl;
        }
      }  
    }

    else if (torqueControlType == "PassivityPIDTerm") {

      m_torqueControlType = torque_control::TorqueFeedbackTerm::PassivityPIDTerm;
      o_strm << "selecting the integral term type of feedback control ";

      i_strm >> m_beta >> m_lambda >> m_mu >> m_sigma >> m_cis;
      o_strm << "by using the gains: beta = " << m_beta << ", lambda = " << m_lambda << ", mu = " << m_mu << ", sigma = " << m_sigma << ", cis = " << m_cis << std::endl;
    }

    else {

      m_torqueControlType = torque_control::TorqueFeedbackTerm::None;
      o_strm << "not using any torque feedback term" << std::endl;
    }
  
    return true;
  }
  else {
    
    o_strm << "WARNING! cannot define the integral term once the MultiContactMotionSolver has been created" << std::endl;

    return false;
  }
}
