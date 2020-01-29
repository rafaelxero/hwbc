/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Surface.h>
#include <mc_rtc/logging.h>
#include <mc_solver/KinematicsConstraint.h>
#include <mc_solver/QPSolver.h>

#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Form.h>

#include <Tasks/Bounds.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/FA.h>

#include <mc_rbdyn/RobotModule.h>
//#include <mc_rbdyn/stance.h>
#include <mc_rbdyn/Surface.h>

#include <mc_rtc/logging.h>

#include <mc_solver/KinematicsConstraint.h>

#include <mc_tasks/MetaTask.h>

#include <time.h>

namespace
{

std::vector<mc_solver::ContactMsg> contactsMsgFromContacts
  (const mc_rbdyn::Robots & robots, const std::vector<mc_rbdyn::Contact> & contacts)
{
  std::vector<mc_solver::ContactMsg> res;

  for(const auto & c : contacts)
  {
    const auto & r1 = robots.robot(c.r1Index());
    const auto & r2 = robots.robot(c.r2Index());

    unsigned int r1BodyIndex = r1.bodyIndexByName(c.r1Surface()->bodyName());
    unsigned int r2BodyIndex = r2.bodyIndexByName(c.r2Surface()->bodyName());

    sva::PTransformd X_0_b1 = r1.mbc().bodyPosW[r1BodyIndex];
    sva::PTransformd X_0_b2 = r2.mbc().bodyPosW[r2BodyIndex];
    sva::PTransformd X_b1_b2 = X_0_b2*X_0_b1.inv();

    mc_solver::ContactMsg msg;
    msg.r1_index = static_cast<uint16_t>(c.r1Index());
    msg.r2_index = static_cast<uint16_t>(c.r2Index());
    msg.r1_body = c.r1Surface()->bodyName();
    msg.r2_body = c.r2Surface()->bodyName();
    msg.r1_surface = c.r1Surface()->name();
    msg.r2_surface = c.r2Surface()->name();
    msg.r1_points = const_cast<const mc_rbdyn::Surface&>(*(c.r1Surface())).points();
    msg.X_b1_b2 = X_b1_b2;
    msg.nr_generators = static_cast<uint16_t>(mc_rbdyn::Contact::nrConeGen);
    msg.mu = mc_rbdyn::Contact::defaultFriction;
    res.push_back(msg);
  }

  return res;
}

} // anonymous namespace

namespace mc_solver
{

  
/**
 *  QPSolver
 */
  

QPSolver::QPSolver(std::shared_ptr<mc_rbdyn::Robots> robots, double timeStep)
  : robots_p(robots), timeStep(timeStep), solver(),
    first_run_(true), feedback_(false),
    lambda_switch_(0.0), switch_T_(0.005), switch_trigger(false)  // switch_T_(0.1)
{
  solver = std::make_shared<tasks::qp::QPSolver>();
  mbcs_calc_ = std::make_shared<std::vector<rbd::MultiBodyConfig>>();

  q_old_ = robot().mbc().q;
  alpha_old_ = robot().mbc().alpha;
  
  if(timeStep <= 0)
  {
    LOG_ERROR_AND_THROW(std::invalid_argument, "timeStep has to be > 0! timeStep = " << timeStep)
  }

  elapsed_ = {{"updateCurrentState", 0},
              {"solve", 0}};
}

QPSolver::QPSolver(double timeStep)
: QPSolver{std::make_shared<mc_rbdyn::Robots>(), timeStep}
{
}

void QPSolver::addConstraintSet(ConstraintSet & cs)
{
  cs.addToSolver(robots().mbs(), *solver);
}

void QPSolver::removeConstraintSet(ConstraintSet & cs)
{
  cs.removeFromSolver(*solver);
}

void QPSolver::addTask(tasks::qp::Task * task)
{
  solver->addTask(robots().mbs(), task);
}

void QPSolver::addTask(mc_tasks::MetaTask * task)
{
  if(std::find(metaTasks_.begin(), metaTasks_.end(), task) == metaTasks_.end())
  {
    metaTasks_.push_back(task);
    task->addToSolver(*this);
    if(logger_)
    {
      task->addToLogger(*logger_);
    }
    LOG_INFO("Added task " << task->name())
  }
}

void QPSolver::removeTask(tasks::qp::Task * task)
{
  solver->removeTask(task);
  shPtrTasksStorage.erase(std::remove_if(
    shPtrTasksStorage.begin(), shPtrTasksStorage.end(),
    [task](const std::shared_ptr<void> & p)
    {
      return task == p.get();
    }), shPtrTasksStorage.end());
}

void QPSolver::removeTask(mc_tasks::MetaTask * task)
{
  auto it = std::find(metaTasks_.begin(), metaTasks_.end(), task);
  if(it != metaTasks_.end())
  {
    metaTasks_.erase(it);
    task->removeFromSolver(*this);
    shPtrTasksStorage.erase(std::remove_if(
      shPtrTasksStorage.begin(), shPtrTasksStorage.end(),
      [task](const std::shared_ptr<void> & p)
      {
        return task == p.get();
      }), shPtrTasksStorage.end());
    if(logger_)
    {
      task->removeFromLogger(*logger_);
    }
    LOG_INFO("Removed task " << task->name())
  }
}

bool QPSolver::hasConstraint(const tasks::qp::Constraint* constraint)
{
  return solver->hasConstraint(constraint);
}

std::pair<int, const tasks::qp::BilateralContact&> QPSolver::contactById(const tasks::qp::ContactId & id) const
{
  const std::vector<tasks::qp::BilateralContact> & contacts = solver->data().allContacts();
  for(size_t i = 0; i < contacts.size(); ++i)
  {
    if(id == contacts[i].contactId)
    {
      return std::pair<int, const tasks::qp::BilateralContact&>(i, contacts[i]);
    }
  }
  // Of course this ref has no value here...
  return std::pair<int, const tasks::qp::BilateralContact&>(-1, tasks::qp::BilateralContact());
}

Eigen::VectorXd QPSolver::lambdaVec(int cIndex) const
{
  return solver->lambdaVec(cIndex);
}

void QPSolver::setContacts(const std::vector<mc_rbdyn::Contact> & contacts)
{
  uniContacts.clear();
  biContacts.clear();
  qpRes.contacts.clear();

  for(const mc_rbdyn::Contact & c : contacts)
  {
    QPContactPtr qcptr = c.taskContact(*robots_p);
    if(qcptr.unilateralContact)
    {
      uniContacts.push_back(tasks::qp::UnilateralContact(*qcptr.unilateralContact));
      delete qcptr.unilateralContact;
      qcptr.unilateralContact = 0;
    }
    else
    {
      biContacts.push_back(tasks::qp::BilateralContact(*qcptr.bilateralContact));
      delete qcptr.bilateralContact;
      qcptr.bilateralContact = 0;
    }
  }

  solver->nrVars(robots_p->mbs(), uniContacts, biContacts);
  const tasks::qp::SolverData & data = solver->data();
  qpRes.contacts = contactsMsgFromContacts(*robots_p, contacts);
  qpRes.contacts_lambda_begin.clear();
  for(int i = 0; i < data.nrContacts(); ++i)
  {
    qpRes.contacts_lambda_begin.push_back(data.lambdaBegin(i) - data.lambdaBegin());
  }
  updateConstrSize();
}

bool QPSolver::run(FeedbackType fType)
{
  bool success = false;
  switch(fType)
  {
    case FeedbackType::None:
      success = runOpenLoop();
      break;
    case FeedbackType::Joints:
      success = runJointsFeedback(false);
      break;
    case FeedbackType::JointsWVelocity:
      success = runJointsFeedback(true);
      break;
    default:
      LOG_ERROR("FeedbackType set to unknown value")
      break;
  }
  if(success)
  {
    __fillResult();
  }
  return success;
}

bool QPSolver::runOpenLoop()
{
  for(auto & t : metaTasks_)
  {
    t->update();
  }
  if(solver->solveNoMbcUpdate(robots_p->mbs(), robots_p->mbcs()))
  {
    for(size_t i = 0; i < robots_p->mbs().size(); ++i)
    {
      rbd::MultiBody & mb = robots_p->mbs()[i];
      rbd::MultiBodyConfig & mbc = robots_p->mbcs()[i];
      if(mb.nrDof() > 0)
      {
        solver->updateMbc(mbc, static_cast<int>(i));
        rbd::eulerIntegration(mb, mbc, timeStep);
        rbd::forwardKinematics(mb, mbc);
        rbd::forwardVelocity(mb, mbc);
      }
    }
    return true;
  }
  return false;
}

bool QPSolver::runJointsFeedback(bool wVelocity)
{
  if(control_q_.size() < robots().size())
  {
    prev_encoders_.resize(robots().size());
    encoders_alpha_.resize(robots().size());
    control_q_.resize(robots().size());
    control_alpha_.resize(robots().size());
  }
  for(size_t i = 0; i < robots().size(); ++i)
  {
    auto & robot = robots().robot(i);
    control_q_[i] = robot.mbc().q;
    control_alpha_[i] = robot.mbc().alpha;
    const auto & encoders = robot.encoderValues();
    if(encoders.size())
    {
      // FIXME Not correct for every joint types
      if(prev_encoders_[i].size() == 0)
      {
        prev_encoders_[i] = robot.encoderValues();
        encoders_alpha_[i].resize(prev_encoders_[i].size());
        if(logger_ && i == 0)
        {
          logger_->addLogEntry("alphaIn", [this]() -> const std::vector<double> & { return encoders_alpha_[0]; });
        }
      }
      for(size_t j = 0; j < encoders.size(); ++j)
      {
        encoders_alpha_[i][j] = (encoders[j] - prev_encoders_[i][j]) / timeStep;
        prev_encoders_[i][j] = encoders[j];
      }
      for(size_t j = 0; j < robot.refJointOrder().size(); ++j)
      {
        const auto & jN = robot.refJointOrder()[j];
        if(!robot.hasJoint(jN))
        {
          continue;
        }
        auto jI = robot.jointIndexByName(jN);
        robot.mbc().q[jI][0] = encoders[j];
        if(wVelocity)
        {
          robot.mbc().alpha[jI][0] = encoders_alpha_[i][j];
        }
      }
      robot.forwardKinematics();
      robot.forwardVelocity();
      robot.forwardAcceleration();
    }
  }
  for(auto & t : metaTasks_)
  {
    t->update();
  }
  if(solver->solveNoMbcUpdate(robots_p->mbs(), robots_p->mbcs()))
  {
    for(size_t i = 0; i < robots_p->mbs().size(); ++i)
    {
      auto & robot = robots().robot(i);
      robot.mbc().q = control_q_[i];
      robot.mbc().alpha = control_alpha_[i];
      if(robot.mb().nrDof() > 0)
      {
        solver->updateMbc(robot.mbc(), static_cast<int>(i));
        robot.eulerIntegration(timeStep);
        robot.forwardKinematics();
        robot.forwardVelocity();
        robot.forwardAcceleration();
      }
    }
    return true;
  }
  return false;
}

bool QPSolver::runClosedLoop(std::shared_ptr<mc_rbdyn::Robots> real_robots)
{
  // =============================
  // 1 - Save old integrator state
  // =============================
  std::vector<std::vector<double>> oldQ(robot().mbc().q);
  std::vector<std::vector<double>> oldQd(robot().mbc().alpha);

  // Set robot state to estimator
  robot().mbc().q = real_robots->robot().mbc().q;
  robot().mbc().alpha = real_robots->robot().mbc().alpha;

  // COMPUTE QP on estimated robot
  for(auto & t : metaTasks_)
  {
    t->update();
  }
  bool success = solver->solveNoMbcUpdate(robots().mbs(), robots().mbcs());
  solver->updateMbc(robot().mbc(), static_cast<int>(robots().robotIndex()));

  // Apply computed acceleration to integrator state
  robot().mbc().q = oldQ;
  robot().mbc().alpha = oldQd;

  // Integrate Qdd on top of integrator state q, qd
  rbd::eulerIntegration(robot().mb(), robot().mbc(), timeStep);
  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());

  __fillResult();
  return success;
}
  
bool QPSolver::run(bool dummy) // Rafa's version
{
  clock_t time;
  
  for(auto & t : metaTasks_)
  {
    t->update();
  }

  time = clock();
  updateCurrentState();
  elapsed_.at("updateCurrentState") = (int) (clock() - time);

  time = clock();
  bool success = solve();
  elapsed_.at("solve") = (int) (clock() - time);

  return success;
}

void QPSolver::updateCurrentState()
{
  if(first_run_)
  {
    encoder_prev_ = robot().encoderValues();
  }

  const std::vector<double> & encoder = robot().encoderValues();
  const Eigen::Vector3d & pIn = robot().bodySensor().position();
  const Eigen::Quaterniond & qtIn = robot().bodySensor().orientation();
  const Eigen::Vector3d & velIn = robot().bodySensor().linearVelocity();
  const Eigen::Vector3d & rateIn = robot().bodySensor().angularVelocity();

  std::vector<double> encoderVel;

  for (size_t i = 0; i <= encoder.size(); i++)
  {
    encoderVel.push_back((encoder[i] - encoder_prev_[i]) / timeStep);
  }

  encoder_prev_ = encoder;

  std::cout << "Rafa, in QPSolver::updateCurrentState, robot().mbc().q[0][4~6] = " << Eigen::Vector3d(robot().mbc().q[0][4], robot().mbc().q[0][5], robot().mbc().q[0][6]).transpose() << std::endl;

  if(feedback_)
  {
    if (lambda_switch_ < 1.0)
    {
      lambda_switch_ += timeStep / switch_T_;
      lambda_switch_ = std::min(lambda_switch_, 1.0);

      std::cout << "Rafa, in QPSolver::updateCurrentState, lambda_switch_ = " << lambda_switch_ << std::endl;

      Eigen::Quaterniond qt_old(q_old_[0][0], q_old_[0][1], q_old_[0][2], q_old_[0][3]);
      Eigen::Quaterniond qt_res = qt_old.slerp(lambda_switch_, qtIn);
      Eigen::Vector3d p_res = pIn * lambda_switch_ + Eigen::Vector3d(q_old_[0][4], q_old_[0][5], q_old_[0][6]) * (1.0 - lambda_switch_);
      Eigen::Vector3d rate_res = rateIn * lambda_switch_ + Eigen::Vector3d(alpha_old_[0][0], alpha_old_[0][1], alpha_old_[0][2]) * (1.0 - lambda_switch_);
      Eigen::Vector3d vel_res  = velIn * lambda_switch_ + Eigen::Vector3d(alpha_old_[0][3], alpha_old_[0][4], alpha_old_[0][5]) * (1.0 - lambda_switch_);

      std::cout << "Rafa, in QPSolver::updateCurrentState, q_old_[0][4~6] = " << Eigen::Vector3d(q_old_[0][4], q_old_[0][5], q_old_[0][6]).transpose() << std::endl;
      std::cout << "Rafa, in QPSolver::updateCurrentState, pIn = " << pIn.transpose() << std::endl;
      std::cout << "Rafa, in QPSolver::updateCurrentState, p_res = " << p_res.transpose() << std::endl;
      
      robot().mbc().q[0] = {qt_res.w(), qt_res.x(), qt_res.y(), qt_res.z(), p_res.x(), p_res.y(), p_res.z()};
      robot().mbc().alpha[0] = {rate_res.x(), rate_res.y(), rate_res.z(), vel_res.x(), vel_res.y(), vel_res.z()};

      for(size_t i = 0; i < robot().refJointOrder().size(); ++i)
      {
        const auto & jn = robot().refJointOrder()[i];

        if(robot().hasJoint(jn))
        {
          size_t j = robot().jointIndexByName(jn);

	  // std::cout << "Rafa, in QPSolver::updateCurrentState, robot has joint " << jn << "and robot().mbc().q[j].size() = " << robot().mbc().q[j].size() << std::endl;
	  
          if(robot().mb().joint(j).dof() == 0)
            continue;

          robot().mbc().q[j][0] = encoder[i] * lambda_switch_ + q_old_[j][0] * (1.0 - lambda_switch_);
          robot().mbc().alpha[j][0] = encoderVel[i] * lambda_switch_ + alpha_old_[j][0] * (1.0 - lambda_switch_);
        }
      }
    }

    else
    {
      robot().mbc().q[0] = {qtIn.w(), qtIn.x(), qtIn.y(), qtIn.z(), pIn.x(), pIn.y(), pIn.z()};
      robot().mbc().alpha[0] = {rateIn.x(), rateIn.y(), rateIn.z(), velIn.x(), velIn.y(), velIn.z()};

      for(size_t i = 0; i < robot().refJointOrder().size(); ++i)
      {
        const auto & jn = robot().refJointOrder()[i];

        if(robot().hasJoint(jn))
        {
          size_t j = robot().jointIndexByName(jn);
          
          if(robot().mb().joint(j).dof() == 0)
            continue;
        
          robot().mbc().q[j][0] = encoder[i];
          robot().mbc().alpha[j][0] = encoderVel[i];
        }
      }
    }
  }

  encoder_prev_ = encoder;

  rbd::forwardKinematics(robot().mb(), robot().mbc());
  rbd::forwardVelocity(robot().mb(), robot().mbc());
  
  if(first_run_)
  {
    (*mbcs_calc_) = robots().mbcs();
    first_run_ = false;
  }

  robot().forwardDynamics();
  robot().feedforwardFriction();
}

bool QPSolver::solve()
{
  bool success = false;

  if(solver->solveNoMbcUpdate(robots_p->mbs(), robots_p->mbcs()))
  {
    for(size_t i = 0; i < robots_p->mbs().size(); ++i)
    {
      rbd::MultiBody & mb = robots_p->mbs()[i];
      rbd::MultiBodyConfig & mbc_real = robots_p->mbcs()[i];
      rbd::MultiBodyConfig & mbc_calc = (*mbcs_calc_)[i];
      if(mb.nrDof() > 0)
      {
        if(!feedback_)
        {
          solver->updateMbc(mbc_real, static_cast<int>(i));
          rbd::eulerIntegration(mb, mbc_real, timeStep);
          mbc_calc = mbc_real;
        }
        else
        {
          solver->updateMbc(mbc_real, static_cast<int>(i));
          solver->updateMbc(mbc_calc, static_cast<int>(i));
          rbd::eulerIntegration(mb, mbc_calc, timeStep);
        }
      }
    }
    
    success = true;
    __fillResult((*mbcs_calc_)[robots().robotIndex()]);
  }
  return success;
}

const QPResultMsg & QPSolver::send(double/*curTime*/)
{
  return qpRes;
}

void QPSolver::__fillResult()
{
  qpRes.zmps.resize(robots().robots().size());
  qpRes.robots_state.resize(robots().robots().size());
  for(unsigned int i = 0; i < robots().robots().size(); ++i)
  {
    const mc_rbdyn::Robot & robot = robots().robot(i);
    auto & q = qpRes.robots_state[i].q;
    auto & alphaVec = qpRes.robots_state[i].alphaVec;
    auto & alphaDVec = qpRes.robots_state[i].alphaDVec;
    for(const auto & j : robot.mb().joints())
    {
      auto jIndex = robot.jointIndexByName(j.name());
      q[j.name()] = robot.mbc().q[jIndex];
      alphaVec[j.name()] = robot.mbc().alpha[jIndex];
      alphaDVec[j.name()] = robot.mbc().alphaD[jIndex];
    }
    // qpRes.robots_state[i].alphaDVec = solver->alphaDVec(static_cast<int>(i));

    qpRes.zmps[i].x = robot.zmpTarget().x();
    qpRes.zmps[i].y = robot.zmpTarget().y();
    qpRes.zmps[i].z = robot.zmpTarget().z();
  }
  qpRes.lambdaVec = solver->lambdaVec();
}

void QPSolver::__fillResult(const rbd::MultiBodyConfig & mbc)
{
  qpRes.robots_state.resize(robots().robots().size());
  for(unsigned int i = 0; i < robots().robots().size(); ++i)
  {
    const mc_rbdyn::Robot & robot = robots().robot(i);
    auto & q = qpRes.robots_state[i].q;
    auto & alphaVec = qpRes.robots_state[i].alphaVec;
    auto & alphaDVec = qpRes.robots_state[i].alphaDVec;
    for(const auto & j : robot.mb().joints())
    {
      auto jIndex = robot.jointIndexByName(j.name());
      q[j.name()] = mbc.q[jIndex];
      alphaVec[j.name()] = mbc.alpha[jIndex];
      alphaDVec[j.name()] = mbc.alphaD[jIndex];
    }
  }
  qpRes.lambdaVec = solver->lambdaVec();
}

const mc_rbdyn::Robot & QPSolver::robot() const
{
  return robots_p->robot();
}
mc_rbdyn::Robot & QPSolver::robot()
{
  return robots_p->robot();
}

mc_rbdyn::Robot & QPSolver::robot(unsigned int idx)
{
  return robots_p->robot(idx);
}
const mc_rbdyn::Robot & QPSolver::robot(unsigned int idx) const
{
  return robots_p->robot(idx);
}

const mc_rbdyn::Robot & QPSolver::env() const
{
  return robots_p->env();
}
mc_rbdyn::Robot & QPSolver::env()
{
  return robots_p->env();
}

const mc_rbdyn::Robots & QPSolver::robots() const
{
  assert(robots_p);
  return *robots_p;
}
mc_rbdyn::Robots & QPSolver::robots()
{
  assert(robots_p);
  return *robots_p;
}

const std::shared_ptr<std::vector<rbd::MultiBodyConfig>> QPSolver::mbcs_calc() const
{
  return mbcs_calc_;
}

const rbd::MultiBodyConfig & QPSolver::mbc_calc() const
{
  return (*mbcs_calc_)[robots().robotIndex()];
}

void QPSolver::updateConstrSize()
{
  solver->updateConstrSize();
}

void QPSolver::updateNrVars()
{
  solver->nrVars(robots_p->mbs(), uniContacts, biContacts);
}

double QPSolver::dt() const
{
  return timeStep;
}

tasks::qp::SolverData & QPSolver::data()
{
  return solver->data();
}

void QPSolver::fillTorque(const mc_solver::DynamicsConstraint& dynamicsConstraint)
{
  fillTorque(dynamicsConstraint.motionConstr.get());
}

void QPSolver::fillTorque(tasks::qp::MotionConstr* motionConstr)
{
  if(solver->hasConstraint(motionConstr))
  {
    motionConstr->computeTorque(solver->alphaDVec(), solver->lambdaVec());
    robot().mbc().jointTorque = rbd::vectorToDof(robot().mb(), motionConstr->torque());
  }
  else
  {
    robot().mbc().jointTorque = rbd::vectorToDof(robot().mb(), Eigen::VectorXd::Zero(robot().mb().nrDof()));
  }

  std::size_t i = robots().robotIndex();
  auto & tau = qpRes.robots_state[i].torque;
  for(const auto & j : robot().mb().joints())
  {
    auto jIndex = robot().jointIndexByName(j.name());
    tau[j.name()] = robot().mbc().jointTorque[jIndex];
  }
}

void QPSolver::enableFeedback(bool fb)
{
  if (!feedback_ && fb)
  {
    switch_trigger = true;
    lambda_switch_ = 0.0;
    q_old_ = robot().mbc().q;
    alpha_old_ = robot().mbc().alpha;
  }

  feedback_ = fb;
}

ElapsedTimeMap & QPSolver::getElapsedTimes()
{
  return elapsed_;
}

boost::timer::cpu_times QPSolver::solveTime()
{
  return solver->solveTime();
}

boost::timer::cpu_times QPSolver::solveAndBuildTime()
{
  return solver->solveAndBuildTime();
}

void QPSolver::logger(std::shared_ptr<mc_rtc::Logger> logger)
{
  if(logger_)
  {
    for(auto t : metaTasks_)
    {
      t->removeFromLogger(*logger_);
    }
  }
  logger_ = logger;
  if(logger_)
  {
    for(auto t : metaTasks_)
    {
      t->addToLogger(*logger_);
    }
  }
}

void QPSolver::gui(std::shared_ptr<mc_rtc::gui::StateBuilder> gui)
{
  if(gui_)
  {
    for(auto t : metaTasks_)
    {
      t->removeFromGUI(*gui_);
    }
  }
  gui_ = gui;
  if(gui_)
  {
    for(auto t : metaTasks_)
    {
      addTaskToGUI(t);
    }
    gui_->addElement({"Contacts", "Add"},
                     mc_rtc::gui::Form("Add contact",
                                       [this](const mc_rtc::Configuration & data) {
                                         LOG_INFO("Add contact " << data("R0") << "::" << data("R0 surface") << "/"
                                                                 << data("R1") << "::" << data("R1 surface"))
                                         auto str2idx = [this](const std::string & rName) {
                                           for(const auto & r : robots())
                                           {
                                             if(r.name() == rName)
                                             {
                                               return r.robotIndex();
                                             }
                                           }
                                           LOG_ERROR("The robot name you provided does not match any in the solver")
                                           return static_cast<unsigned int>(robots().size());
                                         };
                                         unsigned int r0Index = str2idx(data("R0"));
                                         unsigned int r1Index = str2idx(data("R1"));
                                         std::string r0Surface = data("R0 surface");
                                         std::string r1Surface = data("R1 surface");
                                         if(r0Index < robots().size() && r1Index < robots().size())
                                         {
                                           contacts_.push_back({robots(), r0Index, r1Index, r0Surface, r1Surface});
                                           setContacts(contacts_);
                                         }
                                       },
                                       mc_rtc::gui::FormDataComboInput{"R0", true, {"robots"}},
                                       mc_rtc::gui::FormDataComboInput{"R0 surface", true, {"surfaces", "$R0"}},
                                       mc_rtc::gui::FormDataComboInput{"R1", true, {"robots"}},
                                       mc_rtc::gui::FormDataComboInput{"R1 surface", true, {"surfaces", "$R1"}}));
  }
}

void QPSolver::addTaskToGUI(mc_tasks::MetaTask * t)
{
  assert(gui_);
  t->addToGUI(*gui_);
  gui_->addElement({"Tasks", t->name()},
                   mc_rtc::gui::Button("Remove from solver", [this, t]() { this->removeTask(t); }));
}
  
/**
 *  IntglTerm_QPSolver
 */

IntglTerm_QPSolver::IntglTerm_QPSolver(std::shared_ptr<mc_rbdyn::Robots> robots, double timeStep,
                                       torque_control::IntegralTerm::IntegralTermType intTermType,
                                       torque_control::IntegralTerm::VelocityGainType velGainType,
                                       double lambda, double phiSlow, double phiFast, double fastFilterWeight)
  : QPSolver(robots, timeStep)
{
  fbTerm_ = std::make_shared<torque_control::IntegralTerm>(robots->mbs(),
                                                           robots->robotIndex(),
                                                           robots->robot().fd(),
                                                           intTermType, velGainType, lambda,
                                                           phiSlow, phiFast,
                                                           fastFilterWeight , timeStep );
  elapsed_.insert({"computeFbTerm", 0});
  elapsed_.insert(fbTerm_->getElapsedTimes().begin(), fbTerm_->getElapsedTimes().end());
}

IntglTerm_QPSolver::IntglTerm_QPSolver(double timeStep,
                                       torque_control::IntegralTerm::IntegralTermType intTermType,
                                       torque_control::IntegralTerm::VelocityGainType velGainType,
                                       double lambda, double phiSlow, double phiFast, double fastFilterWeight)
  : QPSolver(timeStep)
{
  fbTerm_ = std::make_shared<torque_control::IntegralTerm>(robots().mbs(),
                                                           robots().robotIndex(), robot().fd(),
                                                           intTermType, velGainType, lambda, phiSlow,
                                                           phiFast,fastFilterWeight, timeStep);
  elapsed_.insert({"computeFbTerm", 0});
  elapsed_.insert(fbTerm_->getElapsedTimes().begin(), fbTerm_->getElapsedTimes().end());
}

bool IntglTerm_QPSolver::run(bool dummy)
{
  clock_t time;
  
  for(auto & t : metaTasks_)
  {
    t->update();
  }

  time = clock();
  updateCurrentState();
  elapsed_.at("updateCurrentState") = (int) (clock() - time);
    
  time = clock();

  std::vector<std::vector<double>> sentJointTorques(robot().mb().nrJoints());

  for(size_t i = 0; i < static_cast<int>(robot().mbc().q.size()); ++i)
  {
    sentJointTorques[i].resize(robot().mb().joint(i).dof());
  }

  for(size_t i = 0; i < robot().refJointOrder().size(); ++i)
  {
    const auto & jn = robot().refJointOrder()[i];

    if(robot().hasJoint(jn))
      {
        size_t j = robot().jointIndexByName(jn);

        if(robot().mb().joint(j).dof() == 0)
          continue;

        sentJointTorques[j][0] = robot().jointTorques()[i];
      }
  }

  Eigen::VectorXd sent_torques = rbd::dofToVector(robot().mb(), sentJointTorques);
  Eigen::VectorXd ref_torques = rbd::dofToVector(robot().mb(), robot().mbc().jointTorque);

  Eigen::VectorXd diff_torques = Eigen::VectorXd::Zero(ref_torques.size());

  if (switch_trigger)
  {
    std::cout << "Rafa, in IntglTerm_QPSolver::run, switch_trigger happened !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    
    diff_torques = sent_torques - ref_torques;
    fbTerm_->computeTerm(robot().mb(), robot().mbc(), (*mbcs_calc_)[robots().robotIndex()], diff_torques);
    switch_trigger = false;
  }
  else
  {
    fbTerm_->computeTerm(robot().mb(), robot().mbc(), (*mbcs_calc_)[robots().robotIndex()]);
  }

  std::cout << "Rafa, in IntglTerm_QPSolver::run, sent_torques = " << sent_torques.transpose() << std::endl;
  std::cout << "Rafa, in IntglTerm_QPSolver::run, ref_torques = " << ref_torques.transpose() << std::endl;
  std::cout << "Rafa, in IntglTerm_QPSolver::run, diff_torques = " << diff_torques.transpose() << std::endl;
  
  // fbTerm_->computeTerm(robot().mb(), robot().mbc(), (*mbcs_calc_)[robots().robotIndex()]);
  elapsed_.at("computeFbTerm") = (int) (clock() - time);

  for (ElapsedTimeMap::iterator it = fbTerm_->getElapsedTimes().begin(); it != fbTerm_->getElapsedTimes().end(); it++)
    elapsed_.at(it->first) = it->second;
  
  time = clock();
  bool success = solve();
  elapsed_.at("solve") = (int) (clock() - time);
  
  return success;
}

const std::shared_ptr<torque_control::IntegralTerm> IntglTerm_QPSolver::fbTerm() const
{
  return fbTerm_;
}

/**
 *  IntglTermAntiWindup_QPSolver
 */

IntglTermAntiWindup_QPSolver::IntglTermAntiWindup_QPSolver(std::shared_ptr<mc_rbdyn::Robots> robots, double timeStep,
							   torque_control::IntegralTerm::IntegralTermType intTermType,
							   torque_control::IntegralTerm::VelocityGainType velGainType,
							   double lambda, double perc,
							   const Eigen::Vector3d & maxLinAcc,
							   const Eigen::Vector3d & maxAngAcc,
							   const Eigen::VectorXd & torqueL,
							   const Eigen::VectorXd & torqueU,
                 double phiSlow, double phiFast, 
                 double fastFilterWeight)
  : IntglTerm_QPSolver(robots, timeStep)
{
  fbTerm_ = std::make_shared<torque_control::IntegralTermAntiWindup>(robots->mbs(),
								     robots->robotIndex(),
								     robots->robot().fd(),
								     intTermType, velGainType,
								     lambda, perc,
								     maxLinAcc, maxAngAcc,
								     torqueL, torqueU,
                     phiSlow, phiFast, 
                     fastFilterWeight, timeStep);
}

IntglTermAntiWindup_QPSolver::IntglTermAntiWindup_QPSolver(double timeStep,
							   torque_control::IntegralTerm::IntegralTermType intTermType,
							   torque_control::IntegralTerm::VelocityGainType velGainType,
							   double lambda, double perc,
							   const Eigen::Vector3d & maxLinAcc,
							   const Eigen::Vector3d & maxAngAcc,
							   const Eigen::VectorXd & torqueL,
							   const Eigen::VectorXd & torqueU,
                 double phiSlow, double phiFast, 
                 double fastFilterWeight)
  : IntglTerm_QPSolver(timeStep)
{
  fbTerm_ = std::make_shared<torque_control::IntegralTermAntiWindup>(robots().mbs(),
								     robots().robotIndex(),
								     robot().fd(),
								     intTermType, velGainType,
								     lambda, perc,
								     maxLinAcc, maxAngAcc,
								     torqueL, torqueU,
                     phiSlow, phiFast, 
                     fastFilterWeight, timeStep);
}
 
/**
 *  PassivityPIDTerm_QPSolver
 */
  
PassivityPIDTerm_QPSolver::PassivityPIDTerm_QPSolver(std::shared_ptr<mc_rbdyn::Robots> robots, double timeStep,
                                             double beta, double lambda, double mu, double sigma, double cis)
  : QPSolver(robots, timeStep)
{
  solver = std::make_shared<tasks::qp::PassivityPIDTerm_QPSolver>();
  
  fbTerm_ = std::make_shared<torque_control::PassivityPIDTerm>(robots->mbs(),
                                                               robots->robotIndex(),
                                                               robots->robot().fd(),
                                                               timeStep, beta, lambda, mu, sigma, cis);
}

PassivityPIDTerm_QPSolver::PassivityPIDTerm_QPSolver(double timeStep,
                                             double beta, double lambda, double mu, double sigma, double cis)
  : QPSolver(timeStep)
{
  solver = std::make_shared<tasks::qp::PassivityPIDTerm_QPSolver>();
  
  fbTerm_ = std::make_shared<torque_control::PassivityPIDTerm>(robots().mbs(),
                                                               robots().robotIndex(), robot().fd(),
                                                               timeStep, beta, lambda, mu, sigma, cis);
}

bool PassivityPIDTerm_QPSolver::run(bool dummy)
{
  for(auto & t : metaTasks_)
  {
    t->update();
  }

  updateCurrentState();
  
  fbTerm_->computeTerm(robot().mb(), robot().mbc(), (*mbcs_calc_)[robots().robotIndex()]);
  
  return solve();
}

bool PassivityPIDTerm_QPSolver::solve()
{
  bool success = false;

  std::shared_ptr<tasks::qp::PassivityPIDTerm_QPSolver> passivityPIDTerm_solver = std::dynamic_pointer_cast<tasks::qp::PassivityPIDTerm_QPSolver>(solver);

  if(passivityPIDTerm_solver)
  {
    if(passivityPIDTerm_solver->solveNoMbcUpdate(robots_p->mbs(), robots_p->mbcs(), *mbcs_calc_))
    {
      for(size_t i = 0; i < robots_p->mbs().size(); ++i)
      {
        rbd::MultiBody & mb = robots_p->mbs()[i];
        rbd::MultiBodyConfig & mbc_real = robots_p->mbcs()[i];
        rbd::MultiBodyConfig & mbc_calc = (*mbcs_calc_)[i];
        if(mb.nrDof() > 0)
        {
          if(!feedback_)
          {
            solver->updateMbc(mbc_real, static_cast<int>(i));
            rbd::eulerIntegration(mb, mbc_real, timeStep);
            mbc_calc = mbc_real;
          }
          else
          {
            solver->updateMbc(mbc_real, static_cast<int>(i));
            solver->updateMbc(mbc_calc, static_cast<int>(i));
            rbd::eulerIntegration(mb, mbc_calc, timeStep);

            rbd::forwardKinematics(mb, mbc_calc);
            rbd::forwardVelocity(mb, mbc_calc);
          }
        }
      }

      success = true;
      __fillResult((*mbcs_calc_)[robots().robotIndex()]);
    }
  }
  
  return success;
}

const std::shared_ptr<torque_control::PassivityPIDTerm> PassivityPIDTerm_QPSolver::fbTerm() const
{
  return fbTerm_;
}


} // namespace mc_solver
