/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <iostream>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <ctre/Phoenix.h>
#include <math.h>
#include <frc/WPILib.h> 
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include "rev/ColorSensorV3.h"
#include "rev/CANSparkMax.h"
#include <units/units.h>
#include "ToggleButton.h"





class Robot : public frc::TimedRobot {
  frc::XboxController Driver;
  frc::XboxController Manipulator;
  frc::Joystick Manipulator2;
  TalonFX LeftFalcon1;
  TalonFX LeftFalcon2;
  TalonFX RightFalcon1;
  TalonFX RightFalcon2;
  TalonFX Climber1;
  TalonFX Climber2;
  TalonSRX Turret;
  TalonSRX Indexer;
  VictorSPX IndexerWheel;
  rev::CANSparkMax Shooter1;
  rev::CANSparkMax Shooter2;
  //rev::CANSparkMax Climber1;
  rev::CANSparkMax Intake;
  //rev::CANSparkMax FeederWheel;
  //rev::CANSparkMax IndexerFeeder;
  rev::CANSparkMax ColorWheel; 
  rev::CANSparkMax KickerWheel;
  //TalonSRX ColorWheel;
 // frc::Compressor Compressor;
  frc::Solenoid IntakePosition; 
  frc::Solenoid ShooterHood1;
  frc::Solenoid FlipWheel; 
  frc::Solenoid ClimbPosition;
  frc::Solenoid ColorWheelPosition; 
  PigeonIMU Pigeon;
  
    static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
   rev::ColorSensorV3 m_colorSensor{i2cPort};
   frc::Joystick Driver2{2};
  // frc::JoystickButton LeftHand;
  frc::Timer KickerTimer;
  frc::Timer AntiJamTimer;
  frc::Timer AntiJamTimerpt2;
  frc::Timer RevupTimer;
  ToggleButton BumperShotToggle;
  ToggleButton FarShotToggle;
   


 public:
 double Distance;
 double LimelightAngle = 110; //20
 int HoodPosition;
 double ShooterSpeed;
 double HoodAngle;
 double BallAngle; 
 double LimelightHeight = 22.25;
 double TargetHeight = 98.25;
 double PI = 3.1415926535;
 double ypr [3];
 int RightPosition;
 int LeftPosition;
 bool BumperShot;
 int LoopCount;
 bool TargetSpeed;
 
  int A;
  Robot(void):
  //Remotes
  Driver (0),
  Manipulator (1),
  Driver2 (2),
  Manipulator2 (3),
  //CTRE Motor Controllers
  LeftFalcon1 (7),
  LeftFalcon2 (8),
  RightFalcon1 (5),
  RightFalcon2 (6),
  Turret (9),
  Indexer (10),
  Climber1 (13),
  Climber2 (19),
  ColorWheelPosition(5),
  IndexerWheel(20),
  //Rev morot Controllers
  Shooter1 {11, rev::CANSparkMax::MotorType::kBrushless}, //Right
  Shooter2 {12, rev::CANSparkMax::MotorType::kBrushless}, //Left
  //Climber1 {13, rev::CANSparkMax::MotorType::kBrushless},
  Intake {14 , rev::CANSparkMax::MotorType::kBrushless}, //14
  //FeederWheel {15, rev::CANSparkMax::MotorType::kBrushless},
  //IndexerFeeder {16, rev::CANSparkMax::MotorType::kBrushless},
  ColorWheel {17, rev::CANSparkMax::MotorType::kBrushless},
  KickerWheel{18, rev::CANSparkMax::MotorType::kBrushless},
  //Solinoids/Pistons
  IntakePosition(1), //in
  ShooterHood1(2),  //out
  FlipWheel(3),  //out
  ClimbPosition(4),
  //Pigeon on talon
  Pigeon(&Indexer)
  //Compressor(0)
  //Pigeon(10)
  //ColorWheel (17)

  
{}
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void TankDrive();
  void DriveFollowers();
  void Limelight();
  void ColorSensor();
  void Shooting();
  void Climbing();
  void Indexing();
  void Intaking();
  void ColorWheeling();
  void DrivePIDs();
  void ShootingPIDs();
  void ClimbPIDs();
  void TurretPIDsShort();
  void ShooterEquations();
  void StraightLine();
  void TurretPIDsLong();
  void TurretPIDsSuperShort();
  void Pidgeon();
  void ShootingAuto();
  

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string CenterShootThree = "Shoot Three";
  const std::string TrenchShootSix = "Three + Trench Three";
  const std::string ForwardDrive = "Drive Forward";
  const std::string TrenchShootEight = "Three + Trench Five";
  std::string m_autoSelected;
};
