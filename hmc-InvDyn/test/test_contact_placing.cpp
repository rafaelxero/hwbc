// -*- mode: c++; indent-tabs-mode: nil; tab-width: 2; c-basic-offset: 2; -*-

#include <iostream>
#include <Model/HumanoidBody.h>
#include <Model/HumanoidBodyUtil.h>
//#include <ModelRafa/PointContact.h>
#include <Model/ContactBody.h>
#include <Model/PointContacts.h>
#include <Math/Physics.h>

using namespace hrp;
using namespace motion_generator;

int main(int argc, char *argv[])
{
  const char* url = NULL;
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-url") == 0) {
	    url = argv[++i];
    }
  }
  if (url == NULL) {
    std::cerr << "Please specify the URL of the VRML model by using the -url option" << std::endl;
    return 1;
  }

  HumanoidBodyPtr body(new HumanoidBody());
  loadHumanoidBodyFromModelLoader(body, url, argc, argv);

  const double safetymargin = 0.01;

  //PointContact test(body->ankleLink[0], hrp::Vector3(body->footSize.toe - safetymargin, body->footSize.inside - safetymargin, -body->FootToAnkle(2)), 4, 0.7);

  PointContacts r_contacts(body->ankleLink[0]), l_contacts(body->ankleLink[1]);

  r_contacts.addVertex(Vector3(body->footSize.toe-safetymargin,  body->footSize.inside-safetymargin, -body->FootToAnkle(2)), 0.7, 4);
  r_contacts.addVertex(Vector3(-body->footSize.heel+safetymargin,  body->footSize.inside-safetymargin, -body->FootToAnkle(2)), 0.7, 4);
  r_contacts.addVertex(Vector3(-body->footSize.heel+safetymargin, -body->footSize.outside+safetymargin, -body->FootToAnkle(2)), 0.7, 4);
  r_contacts.addVertex(Vector3(body->footSize.toe-safetymargin, -body->footSize.outside+safetymargin, -body->FootToAnkle(2)), 0.7, 4);

  l_contacts.addVertex(Vector3(body->footSize.toe-safetymargin, -body->footSize.inside+safetymargin, -body->FootToAnkle(2)), 0.7, 4);
  l_contacts.addVertex(Vector3(-body->footSize.heel+safetymargin, -body->footSize.inside+safetymargin, -body->FootToAnkle(2)), 0.7, 4);
  l_contacts.addVertex(Vector3(-body->footSize.heel+safetymargin,  body->footSize.outside-safetymargin, -body->FootToAnkle(2)), 0.7, 4);
  l_contacts.addVertex(Vector3(body->footSize.toe-safetymargin,  body->footSize.outside-safetymargin, -body->FootToAnkle(2)), 0.7, 4);
  
  hrp::dvector initial_posture(hrp::dvector::Zero(body->numJoints()));
 
  initial_posture[body->link("RARM_JOINT0")->jointId] = ToRad(45.0);
  initial_posture[body->link("RARM_JOINT1")->jointId] = ToRad(-20.0);
  initial_posture[body->link("RARM_JOINT3")->jointId] = ToRad(-75.0);
  //initial_posture[body->link("RARM_JOINT7")->jointId] = ToRad(20.0);

  initial_posture[body->link("LARM_JOINT0")->jointId] = ToRad(45.0);
  initial_posture[body->link("LARM_JOINT1")->jointId] = ToRad(20.0);
  initial_posture[body->link("LARM_JOINT3")->jointId] = ToRad(-75.0);
  //initial_posture[body->link("LARM_JOINT7")->jointId] = ToRad(20.0);

  initial_posture[body->link("RLEG_JOINT2")->jointId] = ToRad(-26.0);
  initial_posture[body->link("RLEG_JOINT3")->jointId] = ToRad( 50.0);
  initial_posture[body->link("RLEG_JOINT4")->jointId] = ToRad(-24.0);

  initial_posture[body->link("LLEG_JOINT2")->jointId] = ToRad(-26.0);
  initial_posture[body->link("LLEG_JOINT3")->jointId] = ToRad( 50.0);
  initial_posture[body->link("LLEG_JOINT4")->jointId] = ToRad(-24.0);

  body->rootLink()->p(2) = 0.76;
  for (int i = 0; i < body->numJoints(); i++)
    body->joint(i)->q = initial_posture(i);
  body->calcForwardKinematics();

  ContactBody cbody;
  cbody.addContactPrimitive(&r_contacts);
  cbody.addContactPrimitive(&l_contacts);
  cbody.init();
  
  for(int i = 0 ; i < 2 ; i++ ){
    cbody.setContactState(i, 0, true);
    cbody.setContactState(i, 1, true);
    cbody.setContactState(i, 2, true);
    cbody.setContactState(i, 3, true);
  }
  
  std::cout << "total contacts=" << cbody.numContacts() << std::endl;
  
  Matrix6N W(6, cbody.numContacts());
  cbody.updateJacobian(W);
  
  std::cout << "Wrench Matrix = " << W << std::endl;

  /*
  std::cout << "mass = " << body->totalMass() << std::endl;

  hrp::Vector3 m_total_com = hrp::Vector3::Zero();
  hrp::Matrix33 m_total_inr = hrp::Matrix33::Zero();
  std::vector<motion_generator::LinkInr> m_linkInr;

  body->initComInertia(m_linkInr);
  
  body->calcComInertia(m_total_com, m_total_inr, m_linkInr);
  
  std::cout << "M = " << m_linkInr[0].Mass << std::endl;
  std::cout << "CoM = " << m_linkInr[0].Com.transpose() << std::endl;
  std::cout << "Inr = " << m_linkInr[0].Inr << std::endl;

  hrp::Matrix6N m_AG;
  m_AG.setZero(6, body->numJoints() + 6);
  m_AG.block(0, body->numJoints(), 3, 3) = hrp::Matrix33::Identity() * body->totalMass();
  
  //std::cout << "gdl = " << body->numJoints() << std::endl;

  body->setInertiaMatrix(m_AG, m_linkInr, m_total_com, m_total_inr);
  
  std::cout << "A_G = " << std::endl << m_AG.transpose() << std::endl;
  
  hrp::dmatrix m_M;

  //body->rootLink()->vo = hrp::Vector3(0.24, 0.76, 0);
  body->rootLink()->vo = hrp::Vector3(0, 0, 0);
  //body->rootLink()->w  = hrp::Vector3(1, 1, 0);
  body->rootLink()->w  = hrp::Vector3(0, 0, 0);

  body->calcForwardKinematics(true, true);

  body->calcMassMatrix(m_M);

  hrp::dvector b;
  
  body->calcNonlinearTerm(b);
  */
  
  //std::cout << "b: " << b.transpose() << std::endl;
  
  /*
  hrp::Vector3 out_f;
  hrp::Vector3 out_tau;

  hrp::Vector3 dvoorg;
  hrp::Vector3 dworg;
  hrp::Vector3 root_w_x_v;
  hrp::Vector3 g(0, 0, 9.81);

  dvoorg = body->rootLink()->dvo;
  dworg  = body->rootLink()->dw;
  root_w_x_v = body->rootLink()->w.cross(body->rootLink()->vo + body->rootLink()->w.cross(body->rootLink()->p));
  body->rootLink()->dvo = g - root_w_x_v;   // dv = g, dw = 0
  body->rootLink()->dw.setZero();

  body->calcInverseDynamics(body->rootLink(), out_f, out_tau);

  std::cout << "out_f    = " <<   out_f.transpose() << std::endl;
  std::cout << "out_tau  = " << out_tau.transpose() << std::endl;

  out_tau -= body->rootLink()->p.cross(out_f);

  std::cout << "out_tau2 = " << out_tau.transpose() << std::endl;
  */

  /*
  std::cout << "m_M(0:47,  0:9) = " << std::endl << m_M.block(0,0,48,10) << std::endl;
  std::cout << "m_M(0:47, 10:19) = " << std::endl << m_M.block(0,10,48,10) << std::endl;
  std::cout << "m_M(0:47, 20:29) = " << std::endl << m_M.block(0,20,48,10) << std::endl;
  std::cout << "m_M(0:47, 30:39) = " << std::endl << m_M.block(0,30,48,10) << std::endl;
  std::cout << "m_M(0:47, 40:47) = " << std::endl << m_M.block(0,40,48,8) << std::endl;
  */

  //hrp::dmatrix& wrenches = test.wrenchMatrix();
  //std::cout << "Wrenches: " << std::endl;
  //std::cout << wrenches << std::endl;
}
