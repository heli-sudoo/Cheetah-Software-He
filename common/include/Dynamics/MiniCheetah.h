/*! @file MiniCheetah.h
 *  @brief Utility function to build a Mini Cheetah Quadruped object
 *
 * This file is based on MiniCheetahFullRotorModel_mex.m and builds a model
 * of the Mini Cheetah robot.  The inertia parameters of all bodies are
 * determined from CAD.
 *
 */

#ifndef PROJECT_MINICHEETAH_H
#define PROJECT_MINICHEETAH_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Mini Cheetah
 */
template <typename T>
Quadruped<T> buildMiniCheetah() {
  Quadruped<T> cheetah;
  cheetah._robotType = RobotType::MINI_CHEETAH;

  cheetah._bodyLength = 0.19 * 2;
  cheetah._bodyWidth = 0.049 * 2;
  cheetah._bodyHeight = 0.05 * 2;
  cheetah._abadGearRatio = 6;
  cheetah._hipGearRatio = 6;
  cheetah._kneeGearRatio = 9.33;
  cheetah._flywheelRatio = 21; 
  cheetah._abadLinkLength = 0.062;
  cheetah._hipLinkLength = 0.209;
  //cheetah._kneeLinkLength = 0.175;
  //cheetah._maxLegLength = 0.384;
  cheetah._kneeLinkY_offset = 0.004;
  //cheetah._kneeLinkLength = 0.20;
  cheetah._kneeLinkLength = 0.195;
  cheetah._maxLegLength = 0.409;


  cheetah._motorTauMax = 3.f;
  cheetah._batteryV = 24;
  cheetah._motorKT = .05;  // this is flux linkage * pole pairs
  cheetah._motorR = 0.173;
  cheetah._jointDamping = .01;
  cheetah._jointDryFriction = .2;
  //cheetah._jointDamping = .0;
  //cheetah._jointDryFriction = .0;

  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<T> rotorRotationalInertiaZ;
  rotorRotationalInertiaZ << 33, 0, 0,
                             0, 33, 0,
                             0, 0, 63;
  rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> RZ = coordinateRotation<T>(CoordinateAxis::Z, M_PI / 2);

  Mat3<T> rotorRotationalInertiaX =
      RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
      RX * rotorRotationalInertiaZ * RX.transpose();

  //flywheel  Information Contained herein  
  // cheetah._flywheelMass = 0.381 ; //1.69; 
  // With weighted steel plates                  
  // cheetah._flywheelMass = 1.69; 

  cheetah._flywheelRyLocation = Vec3<T> (0.0,    0.0,      0.099 + cheetah._bodyHeight *  0.5 );
  cheetah._flywheelRotorYLocation = Vec3<T> (0.0,    0.0 , 0.099 + cheetah._bodyHeight * 0.5  );

  cheetah._flywheelRxLocation =     Vec3<T> (-0.102,  0.0,  0.099    + (cheetah._bodyHeight * 0.5) );
  cheetah._flywheelRotorXLocation = Vec3<T> (-0.102 , 0.0, 0.099 + (cheetah._bodyHeight * 0.5) );


  T FlyContMass = 1.0815; //mass of the flywheel container without flywheel
  Vec3<T> FlyContCoM( -0.0492,0.0103, 0.0857 + cheetah._bodyHeight *  0.5); 
  T TorsoMass = 3.3;
  Vec3<T>  TorsoCoM(0.,0.,0.);
  T BodyMass = FlyContMass + TorsoMass;
  cheetah._bodyMass = BodyMass;
  Vec3<T>  BodyCoM = (FlyContCoM * FlyContMass + TorsoCoM * TorsoMass ) / BodyMass;
  Mat3<T> FlyContInertia; 
  FlyContInertia << 5857.33,  553.59, -556.29,
                  553.59,  11778.86,  136.18,
                 -556.29,  136.18,  10241.77; 
  FlyContInertia = FlyContInertia * 1e-6;
  // Parallel axis theorem
  Mat3<T> TorsoRotInertia;
  TorsoRotInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
  TorsoRotInertia = TorsoRotInertia * 1e-6;
  
  Vec3<T> BodyToTorso = BodyCoM - TorsoCoM ; 
  Vec3<T> BodyToFlyCont = BodyCoM - FlyContCoM;

  Mat3<T>BodyRotInertia = TorsoRotInertia + InertiaSkewFn(TorsoMass,BodyToTorso) +
                  FlyContInertia + InertiaSkewFn(FlyContMass, BodyToFlyCont);

  Mat3<T> FlyWheelInertia;  

  //flywheel with no steel plates
//   cheetah._flywheelMass = 0.50;
//   //inertia assumed to point thru Z axis 
//   FlyWheelInertia << 797, 0, 0,
//                      0, 797, 0,
//                     0, 0, 1586;

  //flywheel with 3 steel plates 
  cheetah._flywheelMass = 0.924; 
  FlyWheelInertia << 1824, 0, 0,
                  0, 1824, 0,
                  0, 0, 3603;

  // //flywheel with 6 steel plates 
  // cheetah._flywheelMass = 1.35; 
  // FlyWheelInertia << 2882, 0, 0,
  //                 0, 2882, 0,
  //                 0, 0, 5620;

  // cheetah._flywheelMass = 0.997; 
  // FlyWheelInertia << 2281, 0, 0,
  //                 0, 2281, 0,
  //                 0, 0, 4425;





  FlyWheelInertia = FlyWheelInertia  * 1e-6;

  //assumes rotor inertia about the z axis 
  Mat3<T> flywheelRtrRotZInertia; 
  flywheelRtrRotZInertia << 33,0,0,
                           0,33,0,
                           0,0,63;  
  flywheelRtrRotZInertia = 1e-6 * flywheelRtrRotZInertia;
  //align with proper axis
  Mat3<T> flywheelRtrRotYInertia =  RX * flywheelRtrRotZInertia * RX.transpose();
  Mat3<T> flywheelRtrRotXInertia =  RY * flywheelRtrRotZInertia * RY.transpose();
 
  //assuming flywheel aligned with Z axis
  // Mat3<T> flywheelRotZInertia;
  // // Without weighed plates
  // flywheelRotZInertia << 1007, 38, 4,
  //                       38, 875, 12,
  //                       4, 12, 1619;
  // With weighted steel plates                  
  // flywheelRotZInertia << 4768, 40, -2,
  //                       40, 4635, 6,
  //                       -2, 6, 6734;
  // flywheelRotZInertia = flywheelRotZInertia  *  1e-6;
  
  //Conversion to proper axis
  Mat3<T> flywheelRotYInertia = RX * FlyWheelInertia * RX.transpose(); 
  Mat3<T> flywheelRotXInertia = RY * FlyWheelInertia * RY.transpose(); 



  //SPATIAL INERTIAS 

  //Flywheeel's spatial inertia is here
  SpatialInertia<T> flywheelRyInertia(cheetah._flywheelMass,cheetah._flywheelRyLocation,flywheelRotYInertia);
  SpatialInertia<T> flywheelRxInertia(cheetah._flywheelMass,cheetah._flywheelRxLocation,flywheelRotXInertia);
  
  //Flywheeel's rotor spatial inertia is here
  Vec3<T> flywheelYCOM(0.0, 0.0,0.0);
  SpatialInertia<T> flywheelRotorYInertia(0.055, flywheelYCOM,flywheelRtrRotYInertia); //assumes motor and gearbox wears 500g

  // Vec3<T> flywheelXCOM(-0.1, -0.12, -0.0);
  Vec3<T> flywheelXCOM(0.0, 0.0, 0.0);
  SpatialInertia<T> flywheelRotorXInertia(0.055, flywheelXCOM,flywheelRtrRotXInertia ); //assumes motor and gearbox wears 500g


  // spatial inertias
  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 381, 58, 0.45, 58, 560, 0.95, 0.45, 0.95, 444;
  abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(0, 0.036, 0);  // LEFT
  SpatialInertia<T> abadInertia(0.54, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 1983, 245, 13, 245, 2103, 1.5, 13, 1.5, 408;
  hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(0, 0.016, -0.02);
  SpatialInertia<T> hipInertia(0.634, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 6, 0, 0, 0, 248, 0, 0, 0, 245;
  kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<T> kneeCOM(0, 0, -0.061);
  SpatialInertia<T> kneeInertia(0.064, kneeCOM, kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.055, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.055, rotorCOM, rotorRotationalInertiaY);

  // Mat3<T> bodyRotationalInertia;
  // bodyRotationalInertia << 11253, 0, 0, 0, 36203, 0, 0, 0, 42673;
  // bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  // Vec3<T> bodyCOM(0, 0, 0);
  SpatialInertia<T> bodyInertia(cheetah._bodyMass, BodyCoM,
                                BodyRotInertia);

  cheetah._abadInertia = abadInertia;
  cheetah._hipInertia = hipInertia;
  cheetah._kneeInertia = kneeInertia;
  cheetah._abadRotorInertia = rotorInertiaX;
  cheetah._hipRotorInertia = rotorInertiaY;
  cheetah._kneeRotorInertia = rotorInertiaY;
  cheetah._bodyInertia = bodyInertia;

  cheetah._flywheelRxInertia = flywheelRxInertia;
  cheetah._flywheelRyInertia = flywheelRyInertia;

  cheetah._flywheelRotorYInertia= flywheelRotorYInertia;
  cheetah._flywheelRotorXInertia= flywheelRotorXInertia;

  // locations
  cheetah._abadRotorLocation = Vec3<T>(0.125, 0.049, 0);
  cheetah._abadLocation =
      Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
  cheetah._hipLocation = Vec3<T>(0, cheetah._abadLinkLength, 0);
  cheetah._hipRotorLocation = Vec3<T>(0, 0.04, 0);
  cheetah._kneeLocation = Vec3<T>(0, 0, -cheetah._hipLinkLength);
  cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return cheetah;
}

#endif  // PROJECT_MINICHEETAH_H
