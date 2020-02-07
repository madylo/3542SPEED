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





class Robot : public frc::TimedRobot {
  frc::XboxController Driver;
  frc::XboxController Manipulator;
  frc::Joystick Manipulator2;
  TalonFX LeftFalcon1;
  TalonFX LeftFalcon2;
  TalonFX RightFalcon1;
  TalonFX RightFalcon2;
  TalonFX Climber;
  TalonSRX Turret;
  VictorSPX Indexer;
  rev::CANSparkMax Shooter1;
  rev::CANSparkMax Shooter2;
  //rev::CANSparkMax Climber1;
  rev::CANSparkMax Intake;
  rev::CANSparkMax FeederWheel;
  rev::CANSparkMax IndexerFeeder;
  rev::CANSparkMax ColorWheel; 
  //TalonSRX ColorWheel;
  frc::Compressor Compressor;
  frc::Solenoid IntakePosition; 
  frc::Solenoid ShooterHood1;
  frc::Solenoid ShooterHood2; 
  frc::Solenoid ClimbPosition;
  frc::Solenoid ColorWheelPosition; 
  
    static constexpr auto i2cPort = frc::I2C::Port::kOnboard;
   rev::ColorSensorV3 m_colorSensor{i2cPort};
   frc::Joystick Driver2{2};
  // frc::JoystickButton LeftHand;
   


 public:
 double Distance;
 double LimelightAngle = 105;
 int HoodPosition;
 double ShooterSpeed;
 double HoodAngle;
 double BallAngle; 
 double LimelightHeight = 20;
 double TargetHeight = 98.25;
 double PI = 3.1415926535;
 int RightPosition;
 int LeftPosition;
 
  int A;
  Robot(void):
  //Remotes
  Driver (0),
  Manipulator (1),
  Driver2 (2),
  Manipulator2 (3),
  //Motor Controllers
  LeftFalcon1 (7),
  LeftFalcon2 (8),
  RightFalcon1 (5),
  RightFalcon2 (6),
  Turret (9),
  Indexer (10),
  Shooter1 {11, rev::CANSparkMax::MotorType::kBrushless},
  Shooter2 {12, rev::CANSparkMax::MotorType::kBrushless},
  //Climber1 {13, rev::CANSparkMax::MotorType::kBrushless},
  Climber (13),
  Intake {14, rev::CANSparkMax::MotorType::kBrushless},
  FeederWheel {15, rev::CANSparkMax::MotorType::kBrushless},
  IndexerFeeder {16, rev::CANSparkMax::MotorType::kBrushless},
  ColorWheel {17, rev::CANSparkMax::MotorType::kBrushless},
  IntakePosition(1), //in
  ShooterHood1(2),  //out
  ShooterHood2(3),  //out
  ClimbPosition(4),
  ColorWheelPosition(5)
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
  void TurretPIDs();
  void ShooterEquations();
  void StraightLine();
  

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string CenterShootThree = "Shoot Three";
  const std::string TrenchShootSix = "Three + Trench Three";
  const std::string ForwardDrive = "Drive Forward";
  const std::string TrenchShootEight = "Three + Trench Five";
  std::string m_autoSelected;
};
