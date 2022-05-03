#include <iostream>
#include <Utilities/Timer.h>
#include <Utilities/Utilities_print.h>
#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"
#include "../../../../common/FootstepPlanner/GraphSearch.h"
#include "Gait.h"
// #define DRAW_DEBUG_SWINGS
// #define DRAW_DEBUG_PATH

ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters *parameters) : iterationsBetweenMPC(_iterations_between_mpc),
horizonLength(10),
dt(_dt),
// trotting(horizonLength*1, Vec4<int>(0,5,5,0), Vec4<int>(5,5,5,5),"Trotting"),
trotting(int(horizonLength * 1.6), Vec4<int>(0, 8, 8, 0), Vec4<int>(8, 8, 8, 8), "Trotting"),  //9->C上D上
//  trotting(horizonLength, Vec4<int>(0,horizonLength/2,horizonLength/2,0), Vec4<int>(horizonLength/2,horizonLength/2,horizonLength/2,horizonLength/2),"Trotting"),
slowtrotting(int(horizonLength * 2.4), Vec4<int>(0, 12, 6, 18), Vec4<int>(18, 18, 18, 18), "SlowTrotting"), //3->C中D上
bounding(horizonLength, Vec4<int>(5, 5, 0, 0), Vec4<int>(5, 5, 5, 5), "Bounding"), //                              1->C中D下
//bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(3,3,3,3),"Bounding"),
// pronking(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(4,4,4,4),"Pronking"),
pronking(horizonLength, Vec4<int>(5, 5, 0, 0), Vec4<int>(5, 5, 5, 5), "Pronking"), //                                 2->C下D下
jumping(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(2, 2, 2, 2), "Jumping"),
//galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(6,6,6,6),"Galloping"),
//galloping(horizonLength, Vec4<int>(0,2,7,9),Vec4<int>(3,3,3,3),"Galloping"),
galloping(horizonLength, Vec4<int>(0, 3, 6, 9), Vec4<int>(4, 4, 4, 4), "Galloping"),   //
standing(horizonLength, Vec4<int>(0, 0, 0, 0), Vec4<int>(10, 10, 10, 10), "Standing"), //4
// trotRunning(horizonLength, Vec4<int>(0,5,5,0),Vec4<int>(4,4,4,4),"Trot Running"),
trotRunning(int(horizonLength * 1.2), Vec4<int>(0, 6, 6, 0), Vec4<int>(6, 6, 6, 6), "Trot Running"), //                 5->C上D下  lowheight Troting
//  walking(horizonLength, Vec4<int>(0,3,5,8), Vec4<int>(5,5,5,5), "Walking"),
//  walking(horizonLength, Vec4<int>(0,5,3,8), Vec4<int>(5,5,5,5), "Walking"),
//  walking(int(horizonLength*1.2), Vec4<int>(0,6,4,10), Vec4<int>(6,6,6,6), "Walking"),

// walking(int(horizonLength*1.2), Vec4<int>(0,8,4,12), Vec4<int>(12,12,12,12), "Walking"),
// Vec4<int>(the instant that the leg leave the ground), Vec4<int>(how long the foot on the ground>
walking(int(horizonLength * 3), Vec4<int>(0, 20, 10, 30), Vec4<int>(15, 15, 15, 15), "Walking"),    //  6->C下D上

//  walking(int(horizonLength*1.6), Vec4<int>(0,4,8,12), Vec4<int>(12,12,12,12), "Walking"),
walking2(horizonLength, Vec4<int>(0, 5, 5, 0), Vec4<int>(7, 7, 7, 7), "Walking2"),
pacing(horizonLength, Vec4<int>(5, 0, 5, 0), Vec4<int>(5, 5, 5, 5), "Pacing"),
random(horizonLength, Vec4<int>(9, 13, 13, 9), 0.4, "Flying nine thirteenths trot"),
random2(horizonLength, Vec4<int>(8, 16, 16, 8), 0.5, "Double Trot")
{
  _parameters = parameters;
  dtMPC = dt * iterationsBetweenMPC;
  default_iterations_between_mpc = iterationsBetweenMPC;
  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
  setup_problem(dtMPC, horizonLength, 0.4, 180);
  //setup_problem(dtMPC, horizonLength, 0.4, 650); // DH
  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for (int i = 0; i < 4; i++)
    firstSwing[i] = true;

  initSparseMPC();

  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();
}

void ConvexMPCLocomotion::initialize()
{
  for (int i = 0; i < 4; i++)
    firstSwing[i] = true;
  firstRun = true;
}

void ConvexMPCLocomotion::recompute_timing(int iterations_per_mpc)
{
  iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

void ConvexMPCLocomotion::_SetupCommand(ControlFSMData<float> &data)
{
  ///////////////////////////////// XIAO /////////////////////////////////////
  if (gaitNumber == 4)
  // Standing
  {
    if (data._quadruped->_robotType == RobotType::MINI_CHEETAH)
    {
      _body_height = data.userParameters->body_height;
    }

    else
    {
      assert(false);
    }

    float x_vel_cmd, y_vel_cmd;
    float filter(0.1);
    if (data.controlParameters->use_rc)
    { //real
      const rc_control_settings *rc_cmd = data._desiredStateCommand->rcCommand;
      data.userParameters->cmpc_gait = rc_cmd->variable[0];
      // "DesiredStateCommand.cpp" -- joystickLeft[0] = rcCommand->v_des[1]; // Y
      //                              joystickLeft[1] = rcCommand->v_des[0]; // X
      //                              joystickRight[0] = rcCommand->omega_des[2]; // Yaw
      //                              joystickRight[1] = rcCommand->omega_des[1]; // Pitch 
      _pitch_turn_rate = -rc_cmd->omega_des[1];
      _roll_turn_rate = -rc_cmd->omega_des[0];

      // _yaw_turn_rate =0;
      x_vel_cmd = rc_cmd->v_des[0] * 0.5;
      y_vel_cmd = rc_cmd->v_des[1] * 0.5;
      _body_height += rc_cmd->height_variation * 0.08;
      step_height = rc_cmd->step_height * 0.1; //0-0.2
    }
    else
    { //sim
      // Up and Down
      _pitch_turn_rate = data._desiredStateCommand->rightAnalogStick[1] * 3;
      // Left and Right
      _roll_turn_rate = data._desiredStateCommand->rightAnalogStick[0] * 3;
      
      x_vel_cmd = data._desiredStateCommand->leftAnalogStick[1] * 2.5; //+0.07;
      y_vel_cmd = data._desiredStateCommand->leftAnalogStick[0] * 1;
      step_height = data.userParameters->Swing_traj_height;
    }
    _x_vel_des = _x_vel_des * (1 - filter) + x_vel_cmd * filter;
    _y_vel_des = _y_vel_des * (1 - filter) + y_vel_cmd * filter;

    // _yaw_des = data._stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
    _roll_des = data._stateEstimator->getResult().rpy[0] + dt * _roll_turn_rate;
    _pitch_des = data._stateEstimator->getResult().rpy[1] + dt * _pitch_turn_rate;

  }
  else
  // Normal Gait
  {
    if (data._quadruped->_robotType == RobotType::MINI_CHEETAH)
    {
      _body_height = data.userParameters->body_height;
    }
    // else if (data._quadruped->_robotType == RobotType::CHEETAH_3)
    // {
    //   _body_height = 0.35;
    // }
    else
    {
      assert(false);
    }

    float x_vel_cmd, y_vel_cmd;
    float filter(0.1);
    if (data.controlParameters->use_rc)
    { //real
      const rc_control_settings *rc_cmd = data._desiredStateCommand->rcCommand;
      data.userParameters->cmpc_gait = rc_cmd->variable[0];
      _yaw_turn_rate = -rc_cmd->omega_des[2];

      // _yaw_turn_rate =0;
      x_vel_cmd = rc_cmd->v_des[0] * 0.5;
      y_vel_cmd = rc_cmd->v_des[1] * 0.5;
      _body_height += rc_cmd->height_variation * 0.08;
      step_height = rc_cmd->step_height * 0.1; //0-0.2
    }
    else
    { //sim
      _yaw_turn_rate = data._desiredStateCommand->rightAnalogStick[0] * 1.5;
      x_vel_cmd = data._desiredStateCommand->leftAnalogStick[1] * 1.5; //+0.07;
      y_vel_cmd = data._desiredStateCommand->leftAnalogStick[0] * 0.5;
      step_height = data.userParameters->Swing_traj_height;
    }
    _x_vel_des = _x_vel_des * (1 - filter) + x_vel_cmd * filter;
    _y_vel_des = _y_vel_des * (1 - filter) + y_vel_cmd * filter;

    _yaw_des = data._stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;
    _roll_des = 0.;
    _pitch_des = 0.;

  }
  ///////////////////////////////// XIAO /////////////////////////////////////
}

template <>
void ConvexMPCLocomotion::run(ControlFSMData<float> &data)
{
  bool omniMode = false;

  // Command Setup
  _SetupCommand(data);
  gaitNumber = data.userParameters->cmpc_gait;
  //  if(gaitNumber >= 10) {
  //    gaitNumber -= 10;
  //    omniMode = true;
  //  }
  auto &seResult = data._stateEstimator->getResult();
  data._stateEstimator->setRemoterVelocityResult(Vec3<float>(_x_vel_des, _y_vel_des, _yaw_turn_rate));
  // Check if transition to standing
  if (((gaitNumber == 4) && current_gait != 4) || firstRun)
  {
    stand_traj[0] = seResult.position[0];                // Px
    stand_traj[1] = seResult.position[1];                // Py
    // stand_traj[2] = 0.21;
    stand_traj[2] = data.userParameters->body_height;    // Pz
    stand_traj[3] = 0;                                   // Roll
    stand_traj[4] = 0;                                   // Pitch
    stand_traj[5] = seResult.rpy[2];                     // Yaw
    world_position_desired[0] = stand_traj[0];           // Px
    world_position_desired[1] = stand_traj[1];           // Py
  }
  // pick gait
  Gait *gait = &trotting;
  if (gaitNumber == 1)
    gait = &bounding;
  else if (gaitNumber == 2)
    gait = &pronking;
  else if (gaitNumber == 3)
    gait = &slowtrotting; //random; pacing;//
  else if (gaitNumber == 4)
    gait = &standing;
  else if (gaitNumber == 5)
    gait = &trotRunning; //
  else if (gaitNumber == 6)
    gait = &walking;
  else if (gaitNumber == 7)
    gait = &random2;
  else if (gaitNumber == 8)
    gait = &pacing;
  else if (gaitNumber == 10)
    gait = &galloping;
  else if (gaitNumber == 11)
    gait = &jumping;
  else if (gaitNumber == 101)
    gait = &trotting; //
  else if (gaitNumber == 102)
    gait = &trotting; //
  current_gait = gaitNumber;

  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  jumping.setIterations(iterationsBetweenMPC, iterationCounter);

  jumping.setIterations(27 / 2, iterationCounter);

  //printf("[%d] [%d]\n", jumping.get_current_gait_phase(), gait->get_current_gait_phase());
  // check jump trigger
  jump_state.trigger_pressed(jump_state.should_jump(jumping.getCurrentGaitPhase()),
                             data._desiredStateCommand->trigger_pressed);

  // bool too_high = seResult.position[2] > 0.29;
  // check jump action
  if (jump_state.should_jump(jumping.getCurrentGaitPhase()))
  {
    gait = &jumping;
    recompute_timing(27 / 2);
    _body_height = _body_height_jumping;
    currently_jumping = true;
  }
  else
  {
    recompute_timing(default_iterations_between_mpc);
    currently_jumping = false;
  }

  if (_body_height < 0.02)
  {
    _body_height = 0.35;
    // _body_height = 0.3;
  }

  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  
  Vec3<float> v_des_world =
      omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
  Vec3<float> v_robot = seResult.vWorld;

  //pretty_print(v_des_world, std::cout, "v des world");
  
  for (int i = 0; i < 4; i++)
  {
    pFoot[i] = seResult.position +
               seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) +
                                             data._legController->datas[i].p);
  }
  
  ///////////////////////////////// XIAO /////////////////////////////////////
  // Calculate pith angle
  auto pFootFrontZMPC = pFoot[0][2] + pFoot[1][2];
  auto pFootBackZMPC = pFoot[2][2] + pFoot[3][2];
  auto HeightFBMPC = (pFootBackZMPC - pFootFrontZMPC)/2;
  auto calcuPitchMPC = asin(HeightFBMPC/ (0.261*2));
  calcuPitchMPC = calcuPitchMPC>0.3?0.3:calcuPitchMPC;
  calcuPitchMPC = calcuPitchMPC<-0.3?-0.3:calcuPitchMPC;

  //Integral-esque pitch and roll compensation
  if (fabs(v_robot[0]) > .02) //avoid dividing by zero
  {
    // Pitch
    rpy_int[1] += 5 * dt * (calcuPitchMPC - seResult.rpy[1]) / v_robot[0];

    // rpy_int[1] += 5 * dt * (_pitch_des - seResult.rpy[1]) / v_robot[0];
  }
  if (fabs(v_robot[1]) > 0.01)
  {
    // Roll
    rpy_int[0] += dt * (_roll_des - seResult.rpy[0]) / v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  // Define Pitch and Roll
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber != 2); //turn off for pronking
  ///////////////////////////////// XIAO /////////////////////////////////////

  if (gait != &standing)
  {
    // Desired CoM position [Px, Py, Pz]
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }

  // some first time initialization
  if (firstRun)
  {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.rpy[2];

    for (int i = 0; i < 4; i++)
    {

      footSwingTrajectories[i].setHeight(0.05);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
    }
    firstRun = false;
  }

  // foot placement
  for (int l = 0; l < 4; l++)
    swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);

  float side_sign[4] = {-1, 1, -1, 1};
  //  float interleave_y[4] = {-0.08, 0.08, 0.02, -0.02};
  float interleave_y[4] = {0.0, 0.0, 0.02, -0.02};
  //float interleave_gain = -0.13;
  float interleave_gain = 0; //-0.2;
  //float v_abs = std::fabs(seResult.vBody[0]);
  float v_abs = std::fabs(v_des_robot[0]);
  for (int i = 0; i < 4; i++)
  {
    if (firstSwing[i])
    {
      swingTimeRemaining[i] = swingTimes[i];
    }
    else
    {
      swingTimeRemaining[i] -= dt;
    }
    //if(firstSwing[i]) {
    //footSwingTrajectories[i].setHeight(.05);

    footSwingTrajectories[i].setHeight(step_height); //.125);
                                                     //    Vec3<float> offset(0.05, side_sign[i] * .062, 0);

    Vec3<float> offset(0, side_sign[i] * .09, 0);
    //     Vec3<float> offset(0, side_sign[i] * .075, 0);

    if (i < 2)
      offset[0] = 0; // 0.02; //0.03;
    else
      offset[0] = 0.; //0.02; //0.02;//

    //      offset[0]=seResult.vBody[0]*0.01;

    if (gaitNumber == 101)
    {
      if (i < 2)
        offset(0) = 0.2;
      else
        offset(0) = -0.2;
      if (i % 2 == 0)
        offset(1) = -0.15;
      else
        offset(1) = 0.15;
    }
    else if (gaitNumber == 102)
    {
      if (i < 2)
        offset(0) = -0.12;
      else
        offset(0) = 0.12;

      if (i % 2 == 0)
        offset(1) = -0.15;
      else
        offset(1) = 0.15;
    }

    if (gaitNumber == 6) //walk gait
    {
      if (i == 0)
        offset(1) = -0.085 * (1 - fabs(v_des_robot[0]) / 2.0);
      else if (i == 1)
        offset(1) = 0.085 * (1 - fabs(v_des_robot[0]) / 2.0);
    }

    Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);

    pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
    float stance_time = gait->getCurrentStanceTime(dtMPC, i);
    Vec3<float> pYawCorrected =
        coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate * stance_time / 2) * pRobotFrame;

    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected + des_vel * swingTimeRemaining[i]);

    //+ seResult.vWorld * swingTimeRemaining[i];

    float p_rel_max = 0.35f;
    //    float p_rel_max = 0.3f;

    // Using the estimated velocity is correct
    //Vec3<float> des_vel_world = seResult.rBody.transpose() * des_vel;
    float pfx_rel = seResult.vWorld[0] * (.5 + _parameters->cmpc_bonus_swing) * stance_time +
                    .1f * (seResult.vWorld[0] - v_des_world[0]) +
                    (0.5f * seResult.position[2] / 9.81f) * (seResult.vWorld[1] * _yaw_turn_rate);

    if (fabs(pfx_rel) > p_rel_max)
    {
      printf("!!!!!!!!!!!!!!!!out of the max step\n");
    }

    float pfy_rel = seResult.vWorld[1] * .5 * stance_time * dtMPC +
                    .09f * (seResult.vWorld[1] - v_des_world[1]) +
                    (0.5f * seResult.position[2] / 9.81f) * (-seResult.vWorld[0] * _yaw_turn_rate);
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    Pf[0] += pfx_rel;
    Pf[1] += pfy_rel;
    // Pf[2] =pFoot[i][2];
    Pf[2] =  data.userParameters->stair_height_switch>0?0.1:-0.001;
    footSwingTrajectories[i].setFinalPosition(Pf);
  }

  ///////////////////////////////// XIAO /////////////////////////////////////

  auto pFootFrontZ = pFoot[0][2] + pFoot[1][2];
  auto pFootBackZ = pFoot[2][2] + pFoot[3][2];
  auto HeightFB = (pFootBackZ - pFootFrontZ)/2;
  auto calcuPitch = asin(HeightFB/ (0.261*2));
  calcuPitch = calcuPitch>0.3?0.3:calcuPitch;
  calcuPitch = calcuPitch<-0.3?-0.3:calcuPitch;

  // Body height
  auto delta_h = 0.261*2*sin(-calcuPitch);
  delta_h = delta_h>0.025?0.025:delta_h;
  delta_h = delta_h<-0.025?-0.025:delta_h;
  auto calcu_body_height = _body_height + delta_h;
  // std::cout<<"data.H:"<<data._stateEstimator->getResult().position[2]<<std::endl;
  // std::cout<<"\tdata.HH:"<<seResult.position[2]<<std::endl;

  // auto pFootLeft=pFoot[1][2]+pFoot[3][2];
  // auto pFootRight=pFoot[0][2]+pFoot[2][2];
  // auto HeightLR=(pFootLeft-pFootRight)/2;
  // auto calcuRoll=asin(HeightLR/(0.31));
  // calcuRoll=calcuRoll>0.2?0.2:calcuRoll;
  // calcuRoll=calcuRoll<-0.2?-0.2:calcuRoll;

  // std::cout<<"HeightFB:"<<HeightFB<<std::endl;
  // std::cout<<"calcuPitch:"<<calcuPitch<<std::endl;
  // std::cout<<"HeightLR:"<<HeightLR<<std::endl;
  // std::cout<<"calcuRoll:"<<calcuRoll<<std::endl;

  ///////////////////////////////// XIAO /////////////////////////////////////

  // calc gait
  iterationCounter++;

  // load LCM leg swing gains
  Kp << 700, 0, 0,
      0, 700, 0,
      0, 0, 250;
  Kp_stance = 0 * Kp;

  Kd << 7, 0, 0,
      0, 7, 0,
      0, 0, 7;
  // Kd << 7, 0, 0,
  //     0, 14, 0,
  //     0, 0, 14;
  Kd_stance = Kd;
  // gait
  Vec4<float> contactStates = gait->getContactState();
  //  std::cout<<"contactStates:"<<contactStates<<std::endl;
  Vec4<float> swingStates = gait->getSwingState();
  int *mpcTable = gait->getMpcTable();
  updateMPCIfNeeded(mpcTable, data, omniMode);

  //  StateEstimator* se = hw_i->state_estimator;
  Vec4<float> se_contactState(0, 0, 0, 0);

#ifdef DRAW_DEBUG_PATH
  auto *trajectoryDebug = data.visualizationData->addPath();
  if (trajectoryDebug)
  {
    trajectoryDebug->num_points = 10;
    trajectoryDebug->color = {0.2, 0.2, 0.7, 0.5};
    for (int i = 0; i < 10; i++)
    {
      trajectoryDebug->position[i][0] = trajAll[12 * i + 3];
      trajectoryDebug->position[i][1] = trajAll[12 * i + 4];
      trajectoryDebug->position[i][2] = trajAll[12 * i + 5];
      auto *ball = data.visualizationData->addSphere();
      ball->radius = 0.01;
      ball->position = trajectoryDebug->position[i];
      ball->color = {1.0, 0.2, 0.2, 0.5};
    }
  }
#endif

  for (int foot = 0; foot < 4; foot++)
  {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];
    if (swingState > 0) // foot is in swing
    {
      if (firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
      }

#ifdef DRAW_DEBUG_SWINGS
      auto *debugPath = data.visualizationData->addPath();
      if (debugPath)
      {
        debugPath->num_points = 100;
        debugPath->color = {0.2, 1, 0.2, 0.5};
        float step = (1.f - swingState) / 100.f;
        for (int i = 0; i < 100; i++)
        {
          footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState + i * step, swingTimes[foot]);
          debugPath->position[i] = footSwingTrajectories[foot].getPosition();
        }
      }
      auto *finalSphere = data.visualizationData->addSphere();
      if (finalSphere)
      {
        finalSphere->position = footSwingTrajectories[foot].getPosition();
        finalSphere->radius = 0.02;
        finalSphere->color = {0.6, 0.6, 0.2, 0.7};
      }
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
      auto *actualSphere = data.visualizationData->addSphere();
      auto *goalSphere = data.visualizationData->addSphere();
      goalSphere->position = footSwingTrajectories[foot].getPosition();
      actualSphere->position = pFoot[foot];
      goalSphere->radius = 0.02;
      actualSphere->radius = 0.02;
      goalSphere->color = {0.2, 1, 0.2, 0.7};
      actualSphere->color = {0.8, 0.2, 0.2, 0.7};
#endif
      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

      //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
      //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); // velocity dependent friction compensation todo removed
      //hw_i->leg_controller->leg_datas[foot].qd, fsm->main_control_settings.variable[2]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();

      if (!data.userParameters->use_wbc)
      {
        // Update leg control command regardless of the usage of WBIC
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp;
        data._legController->commands[foot].kdCartesian = Kd;
      }
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;

#ifdef DRAW_DEBUG_SWINGS
      auto *actualSphere = data.visualizationData->addSphere();
      actualSphere->position = pFoot[foot];
      actualSphere->radius = 0.02;
      actualSphere->color = {0.2, 0.2, 0.8, 0.7};
#endif

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";

      if (!data.userParameters->use_wbc)
      {
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;

        data._legController->commands[foot].forceFeedForward = f_ff[foot];
        data._legController->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;

        //      footSwingTrajectories[foot]->updateFF(hw_i->leg_controller->leg_datas[foot].q,
        //                                          hw_i->leg_controller->leg_datas[foot].qd, 0); todo removed
        // hw_i->leg_controller->leg_commands[foot].tau_ff += 0*footSwingController[foot]->getTauFF();
      }
      else
      { // Stance foot damping
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = 0. * Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;
      }
      //            cout << "Foot " << foot << " force: " << f_ff[foot].transpose() << "\n";
      se_contactState[foot] = contactState;

      // Update for WBC
      //Fr_des[foot] = -f_ff[foot];
    }
  }

  // se->set_contact_state(se_contactState); todo removed
  data._stateEstimator->setContactPhase(se_contactState);

  // Update For WBC

  if (data.controlParameters->use_rc)
  {
    pBody_des[0] = world_position_desired[0];
    pBody_des[1] = world_position_desired[1];
    // pBody_des[2] = _body_height;
    // XIAO //
    pBody_des[2] = calcu_body_height;

    vBody_des[0] = v_des_world[0];
    vBody_des[1] = v_des_world[1];
    vBody_des[2] = 0.;

    aBody_des.setZero();


    // pBody_RPY_des[0] = 0; //data._desiredStateCommand->data.stateDes(3); // pBody_RPY_des[0]*0.9+0.1*seResult.rpy[0]/2.0;//
    // pBody_RPY_des[1] = data._desiredStateCommand->data.stateDes(4); //real打开pitch
    
    ///////////////////////////////// XIAO /////////////////////////////////////
    if (gaitNumber == 4)
    {
      pBody_RPY_des[0] = 0;       
      pBody_RPY_des[1] = 0;
      pBody_RPY_des[2] = 0;
      
      vBody_Ori_des[0] = _roll_turn_rate;
      vBody_Ori_des[1] = _pitch_turn_rate;
      vBody_Ori_des[2] = 0;
    }

    else
    {
      pBody_RPY_des[0] = 0;       
      pBody_RPY_des[1] = calcuPitch;
      pBody_RPY_des[2] = _yaw_des;

      vBody_Ori_des[0] = 0.;
      vBody_Ori_des[1] = 0.;
      vBody_Ori_des[2] = _yaw_turn_rate;
    }
    ///////////////////////////////// XIAO /////////////////////////////////////

    //contact_state = gait->getContactState();
    contact_state = gait->getContactState();
  }
  else
  {
    // std::cout<<"bodyheight"<<pBody_des[2]<<"rightAnalogStick"<<data._desiredStateCommand->data.stateDes(4)*0.5f<<std::endl;
    pBody_des[0] = world_position_desired[0];
    pBody_des[1] = world_position_desired[1];
    // pBody_des[2] = _body_height + data._desiredStateCommand->data.stateDes(4) * 0.5f; //(-0.4~0.4)*0.5=(-0.2~0.2) 通过摇杆调整身高
    // XIAO //
    pBody_des[2] = calcu_body_height;
    
    // std::cout<<"bodyheight"<<pBody_des[2]<<std::endl;
    vBody_des[0] = v_des_world[0];
    vBody_des[1] = v_des_world[1];
    vBody_des[2] = 0.;

    aBody_des.setZero();
    
    ///////////////////////////////// XIAO /////////////////////////////////////
    if (gaitNumber == 4)
    {
      vBody_Ori_des[0] = _roll_turn_rate;
      vBody_Ori_des[1] = _pitch_turn_rate;
      vBody_Ori_des[2] = 0;

      pBody_RPY_des[0] = 0;
      // pBody_RPY_des[0] = _roll_des;
      pBody_RPY_des[1] = 0;
      // pBody_RPY_des[1] = _pitch_des;
      pBody_RPY_des[2] = 0;

    }
    else
    {
      vBody_Ori_des[0] = 0.;
      vBody_Ori_des[1] = 0.;
      vBody_Ori_des[2] = _yaw_turn_rate;

      //  pBody_RPY_des[0] = 0.;
      //  pBody_RPY_des[1] = 0.;
      pBody_RPY_des[0] =0;   //data._desiredStateCommand->data.stateDes(3); // pBody_RPY_des[0]*0.9+0.1*seResult.rpy[0]/2.0;//
      // pBody_RPY_des[1] = data.userParameters->pitch_angle_byhand;
      // pBody_RPY_des[1] = data._desiredStateCommand->data.stateDes(4); //sim打开摇杆pitch
      pBody_RPY_des[1] = calcuPitch;
      // std::cout<<"pBody_RPY_des[1]"<<pBody_RPY_des[1]<<std::endl;
      pBody_RPY_des[2] = _yaw_des;
    }
    ///////////////////////////////// XIAO /////////////////////////////////////

    //contact_state = gait->getContactState();
    contact_state = gait->getContactState();
  }
    if (gaitNumber == 4)
        for (int i = 0; i < 4; ++i)
         {
            contact_state[i] = true;
        }
  // END of WBC Update
}

template <>
void ConvexMPCLocomotion::run(ControlFSMData<double> &data)
{
  (void)data;
  printf("call to old CMPC with double!\n");
}

void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData<float> &data, bool omniMode)
{
  //iterationsBetweenMPC = 30;
  if ((iterationCounter % iterationsBetweenMPC) == 0)
  {
    auto seResult = data._stateEstimator->getResult();
    float *p = seResult.position.data();

    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
    Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
    //float trajInitial[12] = {0,0,0, 0,0,.25, 0,0,0,0,0,0};

    //printf("Position error: %.3f, integral %.3f\n", pxy_err[0], x_comp_integral);

    if (current_gait == 4)
    {
      ///////////////////////////////// XIAO /////////////////////////////////////
      float trajInitial[12] = {
          _roll_des,
          _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/,
          // Yaw
          (float)stand_traj[5] /*+(float)stateCommand->data.stateDes[11]*/,
          // Px
          (float)stand_traj[0] /*+(float)fsm->main_control_settings.p_des[0]*/,
          // Py
          (float)stand_traj[1] /*+(float)fsm->main_control_settings.p_des[1]*/,
          // Pz
          (float)_body_height /*fsm->main_control_settings.p_des[2]*/,
          0, 0, 0, 0, 0, 0};
      ///////////////////////////////// XIAO /////////////////////////////////////

      for (int i = 0; i < horizonLength; i++)
        for (int j = 0; j < 12; j++)
          trajAll[12 * i + j] = trajInitial[j];
    }

    else
    {
      const float max_pos_error = .1;
      float xStart = world_position_desired[0];
      float yStart = world_position_desired[1];

      if (xStart - p[0] > max_pos_error)
        xStart = p[0] + max_pos_error;
      if (p[0] - xStart > max_pos_error)
        xStart = p[0] - max_pos_error;

      if (yStart - p[1] > max_pos_error)
        yStart = p[1] + max_pos_error;
      if (p[1] - yStart > max_pos_error)
        yStart = p[1] - max_pos_error;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;
      
      ///////////////////////////////// XIAO /////////////////////////////////////
      auto pFootFrontZMPC = pFoot[0][2] + pFoot[1][2];
      auto pFootBackZMPC = pFoot[2][2] + pFoot[3][2];
      auto HeightFBMPC = (pFootBackZMPC - pFootFrontZMPC)/2;
      auto calcuPitchMPC = asin(HeightFBMPC/ (0.261*2));
      calcuPitchMPC = calcuPitchMPC>0.3?0.3:calcuPitchMPC;
      calcuPitchMPC = calcuPitchMPC<-0.3?-0.3:calcuPitchMPC;

      auto delta_hMPC = 0.261*2*sin(-calcuPitchMPC);
      delta_hMPC = delta_hMPC>0.025?0.025:delta_hMPC;
      delta_hMPC = delta_hMPC<-0.025?-0.025:delta_hMPC;
      auto calcu_body_heightMPC = _body_height + delta_hMPC;
      ///////////////////////////////// XIAO /////////////////////////////////////

      float trajInitial[12] = {(float)rpy_comp[0], // 0 roll
                               (float)rpy_comp[1], // 1 pitch
                               _yaw_des,           // 2
                               //yawStart,    // 2
                               xStart,              // 3 Px
                               yStart,              // 4 Py
                               /////////////////// XIAO ///////////////////

                               (float)calcu_body_heightMPC, // 5 Pz - Height
                               
                               /////////////////// XIAO ///////////////////
                               0,                   // 6
                               0,                   // 7
                               _yaw_turn_rate,      // 8
                               v_des_world[0],      // 9
                               v_des_world[1],      // 10
                               0};                  // 11

      for (int i = 0; i < horizonLength; i++)
      {
        for (int j = 0; j < 12; j++)
          trajAll[12 * i + j] = trajInitial[j];

        if (i == 0) // start at current position  TODO consider not doing this
        {
          //trajAll[3] = hw_i->state_estimator->se_pBody[0];
          //trajAll[4] = hw_i->state_estimator->se_pBody[1];
          trajAll[2] = seResult.rpy[2];
        }
        else
        {
          trajAll[12 * i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12 * i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12 * i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        }
      }
    }
    Timer solveTimer;

    if (_parameters->cmpc_use_sparse > 0.5)
    {
      solveSparseMPC(mpcTable, data);
    }
    else
    {
      solveDenseMPC(mpcTable, data);
    }
    //    printf("TOTAL SOLVE TIME: %.3f\n", solveTimer.getMs());
  }
}

void ConvexMPCLocomotion::solveDenseMPC(int *mpcTable, ControlFSMData<float> &data)
{
  auto seResult = data._stateEstimator->getResult();

  // float Q[12] = {1.25, 1.25, 10, 2, 2, 50, 0, 0, 0.3, 1.5, 1.5, 0.2};
  float Q[12] = {1, 1, 10, 2, 4, 40, 0, 0, 1, 1, 1, 1};
  float yaw = seResult.rpy[2];
  float *weights = Q;
  float alpha = 8e-6; //4e-5; // make setting eventually
  //float alpha = 4e-7; // make setting eventually: DH
  float *p = seResult.position.data();
  float *v = seResult.vWorld.data();
  float *w = seResult.omegaWorld.data();
  float *q = seResult.orientation.data();

  float r[12];
  for (int i = 0; i < 12; i++)
    r[i] = pFoot[i % 4][i / 4] - seResult.position[i / 4];

  //printf("current posistion: %3.f %.3f %.3f\n", p[0], p[1], p[2]);

  if (alpha > 1e-4)
  {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }

  Vec3<float> pxy_act(p[0], p[1], 0);
  Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);
  //Vec3<float> pxy_err = pxy_act - pxy_des;
  float pz_err = p[2] - _body_height;
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

  Timer t1;
  dtMPC = dt * iterationsBetweenMPC;
  setup_problem(dtMPC, horizonLength, 0.4, 180);
  //setup_problem(dtMPC,horizonLength,0.4,650); //DH
  update_x_drag(x_comp_integral);
  if (vxy[0] > 0.3 || vxy[0] < -0.3)
  {
    x_comp_integral += _parameters->cmpc_x_drag * pz_err * dtMPC / vxy[0];
  }

  //printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);

  update_solver_settings(_parameters->jcqp_max_iter, _parameters->jcqp_rho,
                         _parameters->jcqp_sigma, _parameters->jcqp_alpha, _parameters->jcqp_terminate, _parameters->use_jcqp);
  //t1.stopPrint("Setup MPC");

  Timer t2;
  //cout << "dtMPC: " << dtMPC << "\n";
  update_problem_data_floats(p, v, q, w, r, yaw, weights, trajAll, alpha, mpcTable);
  //t2.stopPrint("Run MPC");
  // printf("MPC Solve time %f ms\n", t2.getMs());

  for (int leg = 0; leg < 4; leg++)
  {
    Vec3<float> f;
    for (int axis = 0; axis < 3; axis++)
      f[axis] = get_solution(leg * 3 + axis);

    //printf("[%d] %7.3f %7.3f %7.3f\n", leg, f[0], f[1], f[2]);

    f_ff[leg] = -seResult.rBody * f;
    // Update for WBC
    Fr_des[leg] = f;
  }
}

void ConvexMPCLocomotion::solveSparseMPC(int *mpcTable, ControlFSMData<float> &data)
{
  // X0, contact trajectory, state trajectory, feet, get result!
  (void)mpcTable;
  (void)data;
  auto seResult = data._stateEstimator->getResult();

  std::vector<ContactState> contactStates;
  for (int i = 0; i < horizonLength; i++)
  {
    contactStates.emplace_back(mpcTable[i * 4 + 0], mpcTable[i * 4 + 1], mpcTable[i * 4 + 2], mpcTable[i * 4 + 3]);
  }

  for (int i = 0; i < horizonLength; i++)
  {
    for (u32 j = 0; j < 12; j++)
    {
      _sparseTrajectory[i][j] = trajAll[i * 12 + j];
    }
  }

  Vec12<float> feet;
  for (u32 foot = 0; foot < 4; foot++)
  {
    for (u32 axis = 0; axis < 3; axis++)
    {
      feet[foot * 3 + axis] = pFoot[foot][axis] - seResult.position[axis];
    }
  }

  _sparseCMPC.setX0(seResult.position, seResult.vWorld, seResult.orientation, seResult.omegaWorld);
  _sparseCMPC.setContactTrajectory(contactStates.data(), contactStates.size());
  _sparseCMPC.setStateTrajectory(_sparseTrajectory);
  _sparseCMPC.setFeet(feet);
  _sparseCMPC.run();

  Vec12<float> resultForce = _sparseCMPC.getResult();

  for (u32 foot = 0; foot < 4; foot++)
  {
    Vec3<float> force(resultForce[foot * 3], resultForce[foot * 3 + 1], resultForce[foot * 3 + 2]);
    //printf("[%d] %7.3f %7.3f %7.3f\n", foot, force[0], force[1], force[2]);
    f_ff[foot] = -seResult.rBody * force;
    Fr_des[foot] = force;
  }
}

void ConvexMPCLocomotion::initSparseMPC()
{
  Mat3<double> baseInertia;
  baseInertia << 0.07, 0, 0,
      0, 0.26, 0,
      0, 0, 0.242;
  double mass = 9;
  double maxForce = 120;

  std::vector<double> dtTraj;
  for (int i = 0; i < horizonLength; i++)
  {
    dtTraj.push_back(dtMPC);
  }

  Vec12<double> weights;
  weights << 0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2;
  //weights << 0,0,0,1,1,10,0,0,0,0.2,0.2,0;

  _sparseCMPC.setRobotParameters(baseInertia, mass, maxForce);
  _sparseCMPC.setFriction(0.4);
  _sparseCMPC.setWeights(weights, 8e-6);
  _sparseCMPC.setDtTrajectory(dtTraj);

  _sparseTrajectory.resize(horizonLength);
}
