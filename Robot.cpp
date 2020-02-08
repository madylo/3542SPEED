/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>
#include <ctre/Phoenix.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"
#include <frc/WPILib.h>
#include "rev/CANSparkMax.h"
#include "rev/ColorSensorV3.h"

void Robot::RobotInit() 
  {
    //std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("Limelight");
      //double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
    //double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
      //double targetArea =  table->GetNumber("ta",0.0);
      //double targetSkew = table->GetNumber("ts",0.0);
    //Found on the limelight page with the API's (Change variablename & value)
      //nt::NetworkTableInstance::GetDefault("limelight")->GetNumber("<variablename>",0.0);
      //nt::NetworkTableinstance::GetDefault().GetTable("limelight")->PutNumber("<variablename>",<value>);   
    frc::SmartDashboard::PutData("Auto Modes", & m_chooser);
    m_chooser.SetDefaultOption("CenterShootThree", CenterShootThree);
    m_chooser.AddOption("TrenchShootSix", TrenchShootSix);
    m_chooser.AddOption("ForwardDrive", ForwardDrive);
    m_chooser.AddOption("TrenchShootEight", TrenchShootEight);
    frc::SmartDashboard::PutData("Auto Modes", & m_chooser);
    RightFalcon1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor,0,10);
    LeftFalcon1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor,0,10);
    Turret.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,0,10);


    



  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p> This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
void Robot::RobotPeriodic() 
  {
    frc::SmartDashboard::PutNumber("Right Drive Encoder", RightFalcon1.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Left Drive Encoder", LeftFalcon1.GetSelectedSensorPosition()); 
    frc::SmartDashboard::PutNumber("Right Drive Velocity", RightFalcon1.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("Left Drive Velocity", LeftFalcon1.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("Shooter Velocity", Shooter1.Get());
    frc::SmartDashboard::PutNumber( "Case", A);
    frc::SmartDashboard::PutNumber("RightFalconPercentOutput", RightFalcon1.GetMotorOutputPercent());
    frc::SmartDashboard::PutNumber("LeftFalconPercentOutput", LeftFalcon1.GetMotorOutputPercent());
  }

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() 
  {
    m_autoSelected = m_chooser.GetSelected();
    // m_autoSelected = SmartDashboard::GetString(
    //     "Auto Selector", kAutoNameDefault);
    std::cout << "Auto selected: " << m_autoSelected << std::endl;

    if (m_autoSelected == TrenchShootSix) {
      // Custom Auto goes here
    } else {
      // Default Auto goes here
    }
        RightFalcon1.SetSelectedSensorPosition(0);
        LeftFalcon1.SetSelectedSensorPosition(0);
          Robot::DrivePIDs();
          Robot::DriveFollowers();
           A = 0;
  }

void Robot::AutonomousPeriodic() 
  {
    LeftPosition = LeftFalcon1.GetSelectedSensorPosition(1);
    RightPosition = RightFalcon1.GetSelectedSensorPosition(1);

    if (m_autoSelected == ForwardDrive)
      {//Drive Forward
      
        switch (A)
          {
            case 0:
              if(RightFalcon1.GetSelectedSensorPosition()<=130377 and LeftFalcon1.GetSelectedSensorPosition()<=130377) //130377
                {
                  //Go forward 
                    RightFalcon1.Set(ControlMode::Position, 130377);
                    LeftFalcon1.Set(ControlMode::Position, 130377);
                    //A = 1;
                }
            
              if(RightFalcon1.GetSelectedSensorPosition()>=130377 and LeftFalcon1.GetSelectedSensorPosition()>=130377) //half of 130377
                {
                  A = 1;
                }
            break;
            case 1:
              if (RightFalcon1.GetSelectedSensorPosition()>=130377 or LeftFalcon1.GetSelectedSensorPosition()>=130377 )
                {
                  //Stop
                  // RightFalcon1.SetSelectedSensorPosition(0,0,10);
                  // LeftFalcon1.SetSelectedSensorPosition(0,0,10);
                  RightFalcon1.Set(ControlMode::Velocity,0);
                  LeftFalcon1.Set(ControlMode::Velocity, 0);
                  //RightFalcon1.ConfigPeakOutputForward(0,10);
                  //LeftFalcon1.ConfigPeakOutputForward(0,10);
                 if(RightFalcon1.GetSelectedSensorVelocity()<=10 and LeftFalcon1.GetSelectedSensorVelocity()<=10)
                  {
                    //A = 2;
                  }
                }
              else
                {
                  // RightFalcon1.SetSelectedSensorPosition(0,0,10);
                  // LeftFalcon1.SetSelectedSensorPosition(0,0,10);
                  RightFalcon1.Set(ControlMode::Velocity,0);
                  LeftFalcon1.Set(ControlMode::Velocity, 0);
                  if(RightFalcon1.GetSelectedSensorVelocity()<=50 and LeftFalcon1.GetSelectedSensorVelocity()<=50)
                  {
                    RightFalcon1.GetSelectedSensorPosition(0);
                    LeftFalcon1.GetSelectedSensorPosition(0);
                    //A = 2;
                  }
                }
            break;
            case 2:
              /*if(RightFalcon1.GetSelectedSensorPosition()>=130377 and LeftFalcon1.GetSelectedSensorPosition()>=130377)
                {
                  RightFalcon1.Set(ControlMode::Position, 30377);
                  LeftFalcon1.Set(ControlMode::Position, 30377);
                  A = 3;
                }
              else if(RightFalcon1.GetSelectedSensorPosition()<=80377 and LeftFalcon1.GetSelectedSensorPosition()<=80377)
                {
                  A = 3;
                }*/
              if ((RightFalcon1.GetSelectedSensorPosition()<=100 and RightFalcon1.GetSelectedSensorPosition()>=-50000) and (LeftFalcon1.GetSelectedSensorPosition()<=100 and LeftFalcon1.GetSelectedSensorPosition()>=-50000))
                {
                  //Go Backward
                  RightFalcon1.Set(ControlMode::Position, -60000);
                  LeftFalcon1.Set(ControlMode::Position, -60000);
                }
              else if (RightFalcon1.GetSelectedSensorPosition()<=-50000 and LeftFalcon1.GetSelectedSensorPosition()<= -50000)
                {
                  A = 3;
                }
              else
              {
                LeftFalcon1.SetSelectedSensorPosition(0);
                RightFalcon1.SetSelectedSensorPosition(0);
              }
            break;
            case 3:
              if(RightFalcon1.GetSelectedSensorPosition()<=-50000 and LeftFalcon1.GetSelectedSensorPosition()<=-50000)
                {
                  RightFalcon1.Set(ControlMode::PercentOutput, 0);
                  LeftFalcon1.Set(ControlMode::PercentOutput, 0);
                  RightFalcon1.SetSelectedSensorPosition(0,0,10);
                  LeftFalcon1.SetSelectedSensorPosition(0,0,10);

                }
              /*if (RightFalcon1.GetSelectedSensorPosition()<=-100000 and LeftFalcon1.GetSelectedSensorPosition()<=-100000)
                {
                  //Stop motors and reset encoders
                  RightFalcon1.Set(ControlMode::PercentOutput, 0);
                  LeftFalcon1.Set(ControlMode::PercentOutput, 0);
                  RightFalcon1.SetSelectedSensorPosition(0);
                  LeftFalcon1.SetSelectedSensorPosition(0);
                }
              if (RightFalcon1.GetSelectedSensorPosition()<=100 and LeftFalcon1.GetSelectedSensorPosition()<=100)
                {
                  RightFalcon1.Set(ControlMode::Position,0);
                  LeftFalcon1.Set(ControlMode::Position,0);
                  A = 2;
                }*/
            break;
          }  
      }
    if (m_autoSelected == TrenchShootSix) {
        // Custom Auto goes here
      } if(m_autoSelected == CenterShootThree) {
        // Default Auto goes here
        Robot::StraightLine();
      }
  }

void Robot::TeleopInit() 
  {
    Robot::DrivePIDs();
  }
void Robot::TeleopPeriodic() 
  {
    Robot::ColorSensor();
    Robot::DriveFollowers();
    Robot::TankDrive();
    Robot::Limelight();
    Robot::Intaking();
  }

void Robot::TestPeriodic() 
  {
    
  }
void Robot::TankDrive()
  {
   //RightDrive.Set(ControlMode::PercentOutput, Driver2.GetRawAxis(5);
   //LeftDrive.Set(ControlMode::PercentOutput, Driver2.GetRawAxis(1);
   LeftFalcon1.Set(ControlMode::PercentOutput, Driver.GetY(frc::XboxController::kLeftHand)*-.65);
   RightFalcon1.Set(ControlMode::PercentOutput, Driver.GetY(frc::XboxController::kRightHand)*-.65);
  /*if (Driver.GetAButton() ==1 )
  {
       RightFalcon1.Set(ControlMode::Position,40000);
      LeftFalcon1.Set(ControlMode::Position,40000);
  }
  else {
    RightFalcon1.Set(ControlMode::Position, 0 );
    LeftFalcon1.Set(ControlMode::Position, 0);
  } */
  }

void Robot::DriveFollowers()
  {
   //LeftFalcon2.Set(ControlMode::PercentOutput, LeftFalcon1.GetMotorOutputPercent());
   //RightFalcon2.Set(ControlMode::PercentOutput, RightFalcon1.GetMotorOutputPercent());
  RightFalcon2.Follow(RightFalcon1);
  LeftFalcon2.Follow(LeftFalcon1);

  }
void Robot::Limelight()
  {
   std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("Limelight");
    double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
    double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
    double targetArea =  table->GetNumber("ta",0.0);
    double targetSkew = table->GetNumber("ts",0.0); 
    double targetValid = table->GetNumber("tv",0.0);
    double latency = table->GetNumber("tl",0.0);
    double shortSidelength = table->GetNumber("tshort",0.0);
    double longSidelength = table->GetNumber("tlong",0.0);
    double horizontalSidelength = table->GetNumber("thor",0.0);
    double verticalSidelength = table->GetNumber("tvert",0.0);
    double activePipeline = table->GetNumber("getpipe",0.0);
    double threeD_position = table->GetNumber("camtram",0.0);
    double xCorrdinates_arrays = table->GetNumber("tcornx",0.0);
    double yCorrdinates_arrays = table->GetNumber("tcorny",0.0);
    double xRaw_screenspace = table->GetNumber("tx0",0.0);
    double yRaw_screenspace = table->GetNumber("ty0",0.0);
    double area_percent = table->GetNumber("ta0",0.0);
    double skewRotation_degree = table->GetNumber("ts0",0.0);
    double screenspaceX_Raw = table->GetNumber("tx1",0.0);
    double screenspaceY_Raw = table->GetNumber("ty1",0.0);
    double image_Area = table->GetNumber("ta1",0.0);
    double Rotation_skew = table->GetNumber("ts1",0.0);    
    double Y_rawScreenspace = table->GetNumber("tx2",0.0);    
    double x_rawScreenspace = table->GetNumber("ty2",0.0);
    double Area_image = table->GetNumber("ta2",0.0);
    double Skew_rotation = table->GetNumber("ts2",0.0);
    double xCrossheir_A = table->GetNumber("cx0",0.0);
    double yCrossheir_A = table->GetNumber("cy0",0.0);
    double xCrossheir_B = table->GetNumber("cx1",0.0);
    double yCrossheir_B = table->GetNumber("cy1",0.0);
    //nt::NetworkTableInstance::GetDefault().GetTable("limelight")->GetNumber("<tv>",0.0);

    frc::SmartDashboard::PutNumber("targetArea", targetArea);
    frc::SmartDashboard::PutNumber("targetSkew",targetSkew);
    frc::SmartDashboard::PutNumber("tx",targetOffsetAngle_Horizontal);
    frc::SmartDashboard::PutNumber("ty",targetOffsetAngle_Vertical);
    Distance = (TargetHeight-LimelightHeight)/tan((LimelightAngle+targetOffsetAngle_Vertical)*(180/PI));
    if (targetOffsetAngle_Horizontal >= 10)
    {

    }
    else if (targetOffsetAngle_Horizontal <= -10)
    {

    }
    else if (targetOffsetAngle_Horizontal >= 5)
    {

    }
    else if (targetOffsetAngle_Horizontal <= -5)
    {

    }
  }
void Robot::ColorSensor()
  {
    frc::Color detectedColor = m_colorSensor.GetColor();
    double IR = m_colorSensor.GetIR();
    frc::SmartDashboard::PutNumber("Red", detectedColor.red);
    frc::SmartDashboard::PutNumber("Green", detectedColor.green);
    frc::SmartDashboard::PutNumber("Blue", detectedColor.blue);
    frc::SmartDashboard::PutNumber("IR", IR);
    uint32_t proximity = m_colorSensor.GetProximity();
    frc::SmartDashboard::PutNumber("Proximity", proximity);
  }
void Robot::Shooting()
  {
    Robot::ShootingPIDs();
    Robot::TurretPIDs();
    
    //turret = Talon
    Robot::ShooterEquations();
    //Manual Turret
    //Take Out Manual Turret and Manipulator 2  
    if((Turret.GetSelectedSensorPosition()<=3000 or Turret.GetSelectedSensorPosition()>=-3000) and Manipulator2.GetRawButton(1)==0)
      {
      Turret.Set(ControlMode::PercentOutput, Manipulator.GetX(frc::XboxController::kRightHand)*.5);
      }
    //LimelightTurret
    if((Turret.GetSelectedSensorPosition()<=3000 or Turret.GetSelectedSensorPosition()>=-3000) and Manipulator2.GetRawButton(1)==1)
      {
       Robot::Limelight();
      //Other Limelight
      }
    //Setting Hood Position 
    if ((Distance >= 207 and Manipulator.GetBumper(frc::XboxController::kRightHand)== 1) or Manipulator.GetBButton() ==1 )
      {
        HoodPosition = 3 ;
        ShooterSpeed = 4000;
      }
    else if ((Distance <= 207 and Distance>= 120 and Manipulator.GetBumper(frc::XboxController::kRightHand)==1) or Manipulator.GetYButton() ==1)
      {
        HoodPosition = 2 ;
        ShooterSpeed = 4000;
      }
    else if ((Distance <= 120 and Manipulator.GetBumper(frc::XboxController::kRightHand)==1) or Manipulator.GetXButton() ==1 )
    {
        HoodPosition = 1 ;
        ShooterSpeed = 4000;
    }
    else
      {//Defualt Collapsed Position 
        HoodPosition = 3 ;
        ShooterSpeed = 2000;
      }

  }
void Robot::Intaking()
  {
    //Rev
    //Definging PIDs
      double IntakeP = .000006;
      double IntakeI = .0000001;
      double IntakeD = 0;
      
    //Configuring PIDs
      rev::CANPIDController IntakePIDController = Intake.GetPIDController();
      IntakePIDController.SetP(IntakeP);
      IntakePIDController.SetI(IntakeI);
      IntakePIDController.SetD(IntakeD);
      IntakePIDController.SetOutputRange(-.5,.5); 
      rev::CANEncoder IntakeEncoder = Intake.GetEncoder(); 
     // IntakeEncoder.SetPosition() = 0;
      frc::SmartDashboard::PutNumber("Speed", Intake.Get());
      frc::SmartDashboard::PutNumber("IntakeVelocity", IntakeEncoder.GetVelocity());
   
    //Setting the Control Mode
        double  SetPoint = 0.0;
        IntakePIDController.SetReference(SetPoint, rev::ControlType::kVelocity);
      //Intake In
      if (Manipulator.GetAButton()==1 and Manipulator.GetBumper(frc::XboxController::kLeftHand)==0)
        {
         IntakePIDController.SetReference(SetPoint, rev::ControlType::kVelocity);
         Intake.Set(rev::ControlType::kVelocity);
         Intake.Set(SetPoint);
         SetPoint = 2000; 
         Intake.Set(.5);
         //IntakePosition.Set(true);
        }
      //Intake Out
      else if (Manipulator.GetAButton()==0 and Manipulator.GetBumper(frc::XboxController::kLeftHand)==1)
        {
        Intake.Set(-.5);
        //IntakePosition.Set(true);        
        }
      //Intake Off
      else 
        {
          Intake.Set(0);
        //  IntakePosition.Set(false);
        }
        frc::SmartDashboard::PutNumber("SetPoint", SetPoint);
       }
void Robot::Indexing()
  {//talon
      //Constant Turn
      Indexer.Set(ControlMode::PercentOutput, .25);

  }
void Robot::Climbing()
  {
    //Rev
    //Climber1.Set(rev::ControlType::kPosition);
    

  }
void Robot::ColorWheeling()
  {

  } 
void Robot::DrivePIDs()
  {
  //Talon
  //PIDF defined
    double LeftDriveP = .0225;
    double RightDriveP = .0225;
    double LeftDriveI = 0;
    double RightDriveI = 0;
    double LeftDriveD = .3157;
    double RightDriveD = .3157;
    double LeftDriveF = 0; //.5
    double RightDriveF = 0; //.5
  //Inverting Drive
    RightFalcon1.SetInverted(true);
    LeftFalcon1.SetInverted(false);
    RightFalcon2.SetInverted(true);
    LeftFalcon2.SetInverted(false);
  //Setting how to stop
    RightFalcon1.SetNeutralMode(Coast);
    LeftFalcon1.SetNeutralMode(Coast);
    RightFalcon2.SetNeutralMode(Coast);
    LeftFalcon2.SetNeutralMode(Coast);
    LeftFalcon1.ConfigPeakOutputForward(.85);
    RightFalcon1.ConfigPeakOutputForward(.85);
    LeftFalcon1.ConfigPeakOutputReverse(-.85);
    RightFalcon1.ConfigPeakOutputReverse(-.85);
  //Setting up Sensors
    RightFalcon1.SetSensorPhase(false);
    LeftFalcon1.SetSensorPhase(false);
    RightFalcon2.SetSensorPhase(false);
    LeftFalcon2.SetSensorPhase(false);
    RightFalcon1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor,0,10);
    LeftFalcon1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor,0,10);
    RightFalcon2.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor,0,10);
    LeftFalcon2.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor,0,10);
    
  //Configuring PIDF
    RightFalcon1.Config_kP(0,RightDriveP,10);
    LeftFalcon1.Config_kP(0,LeftDriveP,10);
    RightFalcon1.Config_kI(0,RightDriveI,10);
    LeftFalcon1.Config_kI(0,LeftDriveI,10);
    RightFalcon1.Config_kD(0,RightDriveD,10);
    LeftFalcon1.Config_kD(0,LeftDriveD,10);
    RightFalcon1.Config_kF(0,RightDriveF,10);
    LeftFalcon1.Config_kF(0,LeftDriveF,10);
 }
void Robot::ShootingPIDs()
  {
    //Rev
    //Defining PIDs
      double ShooterP = 0;
      double ShooterI = 0;
      double ShooterD = 0;
    //Configuring PIDs
      rev::CANPIDController ShooterPIDController = Shooter1.GetPIDController();
      ShooterPIDController.SetP(ShooterP);
      ShooterPIDController.SetI(ShooterI);
      ShooterPIDController.SetD(ShooterD);
    //Encoder
      Shooter1.GetEncoder();
      Shooter2.GetEncoder();
    //Control Mode
      ShooterPIDController.SetReference(ShooterSpeed, rev::ControlType::kVelocity);
      Shooter1.Set(rev::ControlType::kVelocity);
      Shooter1.Set(ShooterSpeed);
  }
void Robot::TurretPIDs()
  {
    //Talon
    //Defining PID
      double TurretP = 0;
      double TurretI = 0;
      double TurretD = 0;
    //Configuring PIDs
      Turret.Config_kP(0,TurretP,10);
      Turret.Config_kI(0,TurretI,10);
      Turret.Config_kD(0,TurretD,10);


  }
void Robot::ClimbPIDs()
  {
    //Rev
    //Definging PIDs
      double ClimbP = 0;
      double ClimbI = 0;
      double ClimbD = 0;
      double ClimbF = .05;
    //Configuring PIDs
      Climber.Config_kP(0,ClimbP,10);
      Climber.Config_kI(0,ClimbI,10);
      Climber.Config_kD(0,ClimbD,10);
      Climber.Config_kF(0,ClimbF,10);
     // rev::CANPIDController ClimbPIDController = Climber1.GetPIDController();
     /* ClimbPIDController.SetP(ClimbP);
      ClimbPIDController.SetI(ClimbI);
      ClimbPIDController.SetD(ClimbD); */
    //Climb Encoder
      //Climber1.GetEncoder();

  }
void Robot::ShooterEquations()
  {
    //Getting Hood Position
      if (ShooterHood1.Get() == false and ShooterHood2.Get() == false)
        {
          HoodPosition = 1 ;
        }
      else if (ShooterHood1.Get() == false and ShooterHood2.Get() == true)
        {
          HoodPosition = 2 ;
        }
      else if (Shooter1.Get() == true and ShooterHood2.Get() == true)
        {
          HoodPosition = 3 ;
        }
    //Defining Hood Positions and Angles 
      if(HoodPosition == 1)
        {
          HoodAngle = 17;
          BallAngle = 90 - HoodAngle;
        }
      else if (HoodPosition == 2)
        {
          HoodAngle = 24.5;
          BallAngle = 90 - HoodAngle;
        }
      else if (HoodPosition == 3)
        {
          HoodAngle = 62.5;
          BallAngle = 90 - HoodAngle; 
        }
      if (HoodPosition == 1)
      {
        ShooterHood1.Set(false);
        ShooterHood2.Set(false);
      }
      else if (HoodPosition ==2)
      {
        ShooterHood1.Set(true);
        ShooterHood2.Set(false);
      }
      else if (HoodPosition == 3)
      {
        ShooterHood1.Set(true);
        ShooterHood2.Set(true);
      }
      
  }
void Robot::StraightLine()
  {
    LeftFalcon1.Set(ControlMode::Position, LeftPosition+100000);
    RightFalcon1.Set(ControlMode::Position, RightPosition+100000);
  }

#ifndef RUNNING_FRC_TESTS
int main() { 
  return frc::StartRobot<Robot>(); 
  }
#endif
