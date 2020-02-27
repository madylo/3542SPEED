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
#include "Constants.h"
#include "Instrumentation.h"
#include "TrialMotionProfile.h"

void Robot::RobotInit() 
  {

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
    //Turret.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute,0,10);
  

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
       std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
    double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
    double targetArea =  table->GetNumber("ta",0.0);
    double targetSkew = table->GetNumber("ts",0.0); 
    double targetValid = table->GetNumber("tv",0.0);
     Distance = (TargetHeight-LimelightHeight)/tan((LimelightAngle-targetOffsetAngle_Vertical)*(180/PI));
    frc::SmartDashboard::PutNumber("targetArea", targetArea);
    frc::SmartDashboard::PutNumber("targetSkew",targetSkew);
    frc::SmartDashboard::PutNumber("tx",targetOffsetAngle_Horizontal);
    frc::SmartDashboard::PutNumber("ty",targetOffsetAngle_Vertical);
    frc::SmartDashboard::PutNumberArray("YPR", Pigeon.GetYawPitchRoll(ypr));
    frc::SmartDashboard::PutNumber("Yaw",ypr[0]);
    frc::SmartDashboard::PutNumber("Distance", Distance);
    frc::SmartDashboard::PutBoolean("TargetSpeed", TargetSpeed);
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
    Turret.SetSelectedSensorPosition(0,0,10);
    RightFalcon1.SetSelectedSensorPosition(0);
    LeftFalcon1.SetSelectedSensorPosition(0);
    Pigeon.SetYaw(0,10);
    Robot::DrivePIDs();
    Robot::DriveFollowers();
    Robot::ShootingPIDs();
    //Robot::ShootingPIDs();
     A = 0;
    m_autoSelected = m_chooser.GetSelected();
    // m_autoSelected = SmartDashboard::GetString(
    //     "Auto Selector", kAutoNameDefault);
    std::cout << "Auto selected: " << m_autoSelected << std::endl;

    if (m_autoSelected == TrenchShootSix) {
      // Custom Auto goes here
    } else {
      // Default Auto goes here
    }

  }

void Robot::AutonomousPeriodic() 
  {
    Pigeon.GetYawPitchRoll(ypr);
    LeftPosition = LeftFalcon1.GetSelectedSensorPosition(1);
    RightPosition = RightFalcon1.GetSelectedSensorPosition(1);

  if (m_autoSelected == ForwardDrive)
    {
      //Drive Forward
        switch (A)
          {
            case 0:
              if(RightFalcon1.GetSelectedSensorPosition()<=180377 and LeftFalcon1.GetSelectedSensorPosition()<=180377) //130377
                {
                  //Go forward 
                    RightFalcon1.Set(ControlMode::Position, 180377);
                    LeftFalcon1.Set(ControlMode::Position, 180377);
                    Intake.Set(.75);
                    //Indexer.Set(ControlMode::PercentOutput, -.5); 
                    //A = 1;
                }
            
              if(RightFalcon1.GetSelectedSensorPosition()>=180377 and LeftFalcon1.GetSelectedSensorPosition()>=180377) //half of 130377
                {
                  A = 1;
                }
            break;
            case 1:
              if (RightFalcon1.GetSelectedSensorPosition()>=180377 or LeftFalcon1.GetSelectedSensorPosition()>=180377 )
                {
                  //Stop
                    // RightFalcon1.SetSelectedSensorPosition(0,0,10);
                    // LeftFalcon1.SetSelectedSensorPosition(0,0,10);
                    RightFalcon1.Set(ControlMode::Velocity,0);
                    LeftFalcon1.Set(ControlMode::Velocity, 0);
                    Intake.Set(.75);
                    //Indexer.Set(ControlMode::PercentOutput, -.5);
                    //RightFalcon1.ConfigPeakOutputForward(0,10);
                    //LeftFalcon1.ConfigPeakOutputForward(0,10);
                    if(RightFalcon1.GetSelectedSensorVelocity()<=10 and LeftFalcon1.GetSelectedSensorVelocity()<=10)
                      {
                        //A = 2;
                        Intake.Set(.75);
                        //Indexer.Set(ControlMode::PercentOutput, -.5);
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
  if (m_autoSelected == TrenchShootSix)
    {
      switch (A)
        {
          case 0:
            if(RightFalcon1.GetSelectedSensorPosition()<=180377 and LeftFalcon1.GetSelectedSensorPosition()<=180377) //130377
              {
                //Go forward 
                  Turret.Set(ControlMode::Position, 14000);
                  RightFalcon1.Set(ControlMode::Position, 180377);
                  LeftFalcon1.Set(ControlMode::Position, 180377);
                  Intake.Set(.75);
                  //Indexer.Set(ControlMode::PercentOutput, -.5);
                  //A = 1;
              }
            
            if(RightFalcon1.GetSelectedSensorPosition()>=180377 and LeftFalcon1.GetSelectedSensorPosition()>=180377) //half of 130377
              {
                A = 1;
              }
          break;
          case 1:
            if (RightFalcon1.GetSelectedSensorPosition()>=180377 or LeftFalcon1.GetSelectedSensorPosition()>=180377 )
              {
                //Stop
                  // RightFalcon1.SetSelectedSensorPosition(0,0,10);
                  // LeftFalcon1.SetSelectedSensorPosition(0,0,10);
                  RightFalcon1.Set(ControlMode::Velocity,0);
                  LeftFalcon1.Set(ControlMode::Velocity, 0);
                  Intake.Set(.75);
                  Robot::Limelight();
                  //Indexer.Set(ControlMode::PercentOutput, -.5);
                  //RightFalcon1.ConfigPeakOutputForward(0,10);
                  //LeftFalcon1.ConfigPeakOutputForward(0,10);
                 if(RightFalcon1.GetSelectedSensorVelocity()<=10 and LeftFalcon1.GetSelectedSensorVelocity()<=10)
                  {
                    //A = 2;
                    Intake.Set(.75);
                    Robot::Limelight();
                    //Indexer.Set(ControlMode::PercentOutput, -.5);
                  }
              }
            else
              {
                // RightFalcon1.SetSelectedSensorPosition(0,0,10);
                // LeftFalcon1.SetSelectedSensorPosition(0,0,10);
                RightFalcon1.Set(ControlMode::Velocity,0);
                LeftFalcon1.Set(ControlMode::Velocity, 0);
                //Move Turret
                  if(RightFalcon1.GetSelectedSensorVelocity()<=50 and LeftFalcon1.GetSelectedSensorVelocity()<=50)
                  {
                    RightFalcon1.GetSelectedSensorPosition(0);
                    LeftFalcon1.GetSelectedSensorPosition(0);
                    //A = 2;
                  }
              }
            break;
           }
      }
  if(m_autoSelected == CenterShootThree) 
    {
        // Default Auto goes here
        switch (A)
        {
        case 0:
          Robot::Limelight();

          break;
        
        case 1:

          break;
        }
    }
  if(m_autoSelected==TrenchShootEight)
    {
      switch (A)
        {
          //case 0:
            //Shoot
          //break;
          case 0: //1:
            if(RightFalcon1.GetSelectedSensorPosition()<=280377 and LeftFalcon1.GetSelectedSensorPosition()<=280377) //130377
              {
                //Go forward 
                  RightFalcon1.Set(ControlMode::Position, 280377);
                  LeftFalcon1.Set(ControlMode::Position, 280377);
                  Intake.Set(.95);
                  //Indexer.Set(ControlMode::PercentOutput, -.5);
                  //A = 1;
              }
            
            if(RightFalcon1.GetSelectedSensorPosition()>=280377 and LeftFalcon1.GetSelectedSensorPosition()>=280377) 
              {
                A = 1;
              }
          break;
          case 1: //2
            if (RightFalcon1.GetSelectedSensorPosition()>=280377 or LeftFalcon1.GetSelectedSensorPosition()>=280377 )
                {
                  //Stop 
                    // RightFalcon1.SetSelectedSensorPosition(0,0,10);
                    // LeftFalcon1.SetSelectedSensorPosition(0,0,10);
                    RightFalcon1.Set(ControlMode::Velocity,0);
                    LeftFalcon1.Set(ControlMode::Velocity, 0);
                    Intake.Set(.95);
                    //Indexer.Set(ControlMode::PercentOutput, -.5);
                    //RightFalcon1.ConfigPeakOutputForward(0,10);
                    //LeftFalcon1.ConfigPeakOutputForward(0,10);
              if(RightFalcon1.GetSelectedSensorVelocity()<=10 and LeftFalcon1.GetSelectedSensorVelocity()<=10)
                {
                  //resets encoders
                  A = 2;
                  RightFalcon1.SetSelectedSensorPosition(0,0,10);
                  LeftFalcon1.SetSelectedSensorPosition(0,0,10);
                  Intake.Set(.95);
                  //Indexer.Set(ControlMode::PercentOutput, -.5);
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
                      RightFalcon1.SetSelectedSensorPosition(0,0,10);
                      LeftFalcon1.SetSelectedSensorPosition(0,0,10);
                      Intake.Set(.95);
                      //Indexer.Set(ControlMode::PercentOutput, .5);
                      A = 2;
                    }
                }
          break;
          case 2: //3
              if ((RightFalcon1.GetSelectedSensorPosition()<=100 and RightFalcon1.GetSelectedSensorPosition()>=-90000) and (LeftFalcon1.GetSelectedSensorPosition()<=100 and LeftFalcon1.GetSelectedSensorPosition()>=-90000))
                {
                  //Go Backward
                  RightFalcon1.Set(ControlMode::Position, -100000);
                  LeftFalcon1.Set(ControlMode::Position, -100000);
                  Intake.Set(.95);
                  //Indexer.Set(ControlMode::PercentOutput, -.5);
                }
              else if (RightFalcon1.GetSelectedSensorPosition()<=-90000 and LeftFalcon1.GetSelectedSensorPosition()<= -90000)
                {
                  A = 3;
                  Intake.Set(.95);
                  //Indexer.Set(ControlMode::PercentOutput, -.5);
                }
              else 
                {
                  // LeftFalcon1.SetSelectedSensorPosition(0);
                  // RightFalcon1.SetSelectedSensorPosition(0);
                  Intake.Set(.95);
                  //Indexer.Set(ControlMode::PercentOutput, -.5);
                }
          break;
          case 3: //4
            if(RightFalcon1.GetSelectedSensorPosition()<=-90000 and LeftFalcon1.GetSelectedSensorPosition()<=-90000)
              {
                RightFalcon1.Set(ControlMode::Velocity, 0);
                LeftFalcon1.Set(ControlMode::Velocity, 0);
                RightFalcon1.SetSelectedSensorPosition(0,0,10);
                LeftFalcon1.SetSelectedSensorPosition(0,0,10);
                Intake.Set(.95);
                //Indexer.Set(ControlMode::PercentOutput, -.5);

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
  }

void Robot::TeleopInit() 
  {
    Robot::DrivePIDs();
    Robot::ShootingPIDs();
    LoopCount = 0;
    BumperShot = 0;

  }
void Robot::TeleopPeriodic() 
  {
    Robot::ColorSensor();
    Robot::DriveFollowers();
    Robot::TankDrive();
   // Robot::Limelight();
    Robot::Intaking();
    Robot::Shooting();
    Robot::Indexing();
   // Compressor.SetClosedLoopControl(true);
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
   std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
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
    Distance = (TargetHeight-LimelightHeight)/tan((LimelightAngle-targetOffsetAngle_Vertical)*(180/PI));
    // if (targetOffsetAngle_Horizontal >= 10)
    // {
      
    // }
    // else if (targetOffsetAngle_Horizontal <= -10)
    // {

    // }
    // else if (targetOffsetAngle_Horizontal >= 5)
    // {

    // }
    // else if (targetOffsetAngle_Horizontal <= -5)
    // {

    // }
    
    
    Turret.Set(ControlMode::Velocity, (targetOffsetAngle_Horizontal*-280));
    frc::SmartDashboard::PutNumber("VelocityTarget",targetOffsetAngle_Horizontal*280 );
    
    frc::SmartDashboard::PutNumber("targetvalid",targetValid);
    if((targetOffsetAngle_Horizontal<8 and targetOffsetAngle_Horizontal>3 )or (targetOffsetAngle_Horizontal>-8 and targetOffsetAngle_Horizontal<-3))
      {
        Robot::TurretPIDsShort();
      }
    if(targetOffsetAngle_Horizontal>8 or targetOffsetAngle_Horizontal<-8)
      {
        Robot::TurretPIDsLong();
      }
    if(targetOffsetAngle_Horizontal<3 and targetOffsetAngle_Horizontal>-3)
      {
        Robot::TurretPIDsSuperShort();
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
    if(Turret.GetSelectedSensorPosition()>21000)
      {
        Turret.ConfigPeakOutputForward(0,10);
        Turret.ConfigPeakOutputReverse(-.5,10);
      }
    else if(Turret.GetSelectedSensorPosition()<-21000)
      {
        Turret.ConfigPeakOutputReverse(0,10);
        Turret.ConfigPeakOutputForward(.5,10);
      }
    else
      {
        Turret.ConfigPeakOutputForward(.5,10);
        Turret.ConfigPeakOutputReverse(-.5,10);
      }
    frc::SmartDashboard::PutNumber("TurretVelocity",Turret.GetSelectedSensorVelocity());
    //Robot::ShootingPIDs();

    //turret = Talon
    //Robot::ShooterEquations();

    //Manual Turret
    //Take Out Manual Turret and Manipulator 2  
    frc::SmartDashboard::PutNumber("TurretEncoder",Turret.GetSelectedSensorPosition(0));
    if( Manipulator.GetBumper(frc::XboxController::kRightHand)==0)
      {
      Turret.Set(ControlMode::PercentOutput, Manipulator.GetX(frc::XboxController::kRightHand)*-.25);
      }
    //LimelightTurret
    if(Manipulator.GetBumper(frc::XboxController::kRightHand)==1)
      {
       Robot::Limelight();
      //Other Limelight
      }
    //Setting Hood Position 
      // if(BumperShot == 1 and Manipulator.GetBumper(frc::XboxController::kRightHand)==0)
      // {
      //   HoodPosition = 2;
      // }
      // else if(BumperShot == 0 or Manipulator.GetBumper(frc::XboxController::kRightHand)==1)
      // {
      //   HoodPosition = 1;
      // }
        //Rev
    //Defining PIDs
      double ShooterP = .00200;
      double ShooterI = 0.00000;
      double ShooterD = 0.0000000;
      double ShooterIz = 0;
      double ShooterFF = .000205;
    //Configuring PIDs
      rev::CANPIDController ShooterPIDController = Shooter1.GetPIDController();
      rev::CANPIDController Shooter2PIDController = Shooter2.GetPIDController();
      ShooterPIDController.SetP(ShooterP);
      ShooterPIDController.SetI(ShooterI);
      ShooterPIDController.SetD(ShooterD);
      ShooterPIDController.SetFF(ShooterFF);
      ShooterPIDController.SetIZone(ShooterIz);
      ShooterPIDController.SetOutputRange(-.9,9);
      Shooter2PIDController.SetP(ShooterP);
      Shooter2PIDController.SetI(ShooterI);
      Shooter2PIDController.SetD(ShooterD);
      Shooter2PIDController.SetFF(ShooterFF);
      Shooter2PIDController.SetIZone(ShooterIz);
      Shooter2PIDController.SetOutputRange(-.9,9);



    //Current Limits
      Shooter1.SetSmartCurrentLimit(40, 40, 11433);
      Shooter2.SetSmartCurrentLimit(40, 40, 11433);
      Shooter1.SetSecondaryCurrentLimit(40, 20);
      Shooter2.SetSecondaryCurrentLimit(40, 20);
    //Encoder
      rev::CANEncoder ShooterEncoder = Shooter1.GetEncoder();
      rev::CANEncoder ShooterEncoder2 = Shooter2.GetEncoder();
    //  if (Manipulator.GetBumper(frc::XboxController::kLeftHand)==1 )
    // {

    //   Shooter2PIDController.SetReference(-ShooterSpeed, rev::ControlType::kVelocity);
    //   ShooterPIDController.SetReference(ShooterSpeed, rev::ControlType::kVelocity); 
    //   ShooterSpeed=2800;
    //   //3000 = Intiation Line
    // }
    //    else
    //  {   
    //    Shooter1.Set(0);
    //    Shooter2.Set(0);
    //  }
    //       //Indexer.Set(ControlMode::PercentOutput,-1);
    //       // ShooterSpeed = 3000;
        
    //       Shooter1.Set(1);
    //       Shooter2.Set(-1);
    //     }
    // else if (Manipulator.GetAButton()==0)
    //   {
    //     KickerWheel.Set(0);

    //     // Shooter1.Set(rev::ControlType::kVelocity);
    //     // Shooter1.Set(0);
    //     //Indexer.Set(ControlMode::PercentOutput,0);
    //     // ShooterSpeed = 0;
    //   }
  

      // if (Manipulator.GetBumper(frc::XboxController::kLeftHand)==1)
      // {
      //   Shooter1.Set(1);
      //   Shooter2.Set(-1);
      // }
      // else
      // {
      //   Shooter1.Set(0);
      //   Shooter2.Set(0);
      // }
      if(Manipulator.GetAButton()==1)
      {
        KickerTimer.Start();
        FlipWheel.Set(false); //in
       // Indexer.Set(ControlMode::PercentOutput,-1);
        if (KickerTimer.Get()>1.0)
        {
          KickerWheel.Set(1);
        }
      }
      else
      {
        FlipWheel.Set(true); //out
        KickerWheel.Set(0);
        KickerTimer.Stop();
       // Indexer.Set(ControlMode::PercentOutput, 0);
      }
      if (BumperShotToggle.ButtonPress(Manipulator.GetBumper(frc::XboxController::kLeftHand)==1))
      {
      Shooter2PIDController.SetReference(-ShooterSpeed, rev::ControlType::kVelocity);
      ShooterPIDController.SetReference(ShooterSpeed, rev::ControlType::kVelocity); 
      ShooterSpeed=3100;
      //3100 far in trench and flapping open
      //2200 bumpershot and flapping open
      //2800
      //3000
      }
      else 
      {
        Shooter1.Set(0);
        Shooter2.Set(0);
      }
      

    frc::SmartDashboard::PutNumber("Current1", Shooter1.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("ShooterVelocity", ShooterEncoder.GetVelocity() );
   // frc::SmartDashboard::PutNumber("Current2", )
   if ((ShooterEncoder.GetVelocity()<= (ShooterSpeed+100)) and (ShooterEncoder.GetVelocity()>= (ShooterSpeed-100)))
   {
     TargetSpeed = true;
   }
   else
   {
     TargetSpeed = false;
   }

  }
void Robot::ShootingAuto()
  {
    Robot::ShootingPIDs();
    //Turret Restraints
      if(Turret.GetSelectedSensorPosition()>21000)
        {
          Turret.ConfigPeakOutputForward(0,10);
          Turret.ConfigPeakOutputReverse(-.5,10);
        }
      else if(Turret.GetSelectedSensorPosition()<-21000)
        {
          Turret.ConfigPeakOutputReverse(0,10);
          Turret.ConfigPeakOutputForward(.5,10);
        }
      else
        {
          Turret.ConfigPeakOutputForward(.5,10);
          Turret.ConfigPeakOutputReverse(-.5,10);
        }
        //Setting velocity for shooter
      rev::CANPIDController ShooterPIDController = Shooter1.GetPIDController();
      rev::CANPIDController Shooter2PIDController = Shooter2.GetPIDController();
      Shooter2PIDController.SetReference(-ShooterSpeed, rev::ControlType::kVelocity);
      ShooterPIDController.SetReference(ShooterSpeed, rev::ControlType::kVelocity); 
      ShooterSpeed = 2800;
      if(ShooterSpeed >= 2700 and ShooterSpeed < 2900)
        {
          FlipWheel.Set(true);
          Indexer.Set(ControlMode::PercentOutput, -1);
          KickerWheel.Set(1);
            
        }
    //Setting Shooter Velocity
      
      // if (Driver.GetXButton()==1)
      //   {
      //     KickerWheel.Set(.80);
      //   }
      // else if(Driver.GetXButton()==0)
      //   {
      //     KickerWheel.Set(0);
      //   }

  }
void Robot::Intaking()
  {
    //Rev
    //Definging PIDs
      double IntakeP = .000006;
      double IntakeI = .0000001;
      double IntakeD = 0;
    //Current Limit
      Intake.SetSmartCurrentLimit(25, 30, 20000);
      Intake.SetSecondaryCurrentLimit(30, 20);
      
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
       // IntakePIDController.SetReference(SetPoint, rev::ControlType::kVelocity);
      //Intake In
      if (Driver.GetBumper(frc::XboxController::kRightHand)==1 and Driver.GetBumper(frc::XboxController::kLeftHand)==0)
        {
        //  IntakePIDController.SetReference(SetPoint, rev::ControlType::kVelocity);
        //  Intake.Set(rev::ControlType::kVelocity);
        //  Intake.Set(SetPoint);
        //  SetPoint = 2000; 
         Intake.Set(.6);
         IntakePosition.Set(true);
         //IndexerWheel.Set(ControlMode::PercentOutput, -.5);
        // Indexer.Set(ControlMode::PercentOutput, -.4);
        }
      //Intake Out
      else if (Driver.GetBumper(frc::XboxController::kRightHand)==0 and Driver.GetBumper(frc::XboxController::kLeftHand)==1)
        {
        Intake.Set(-.6);
        IntakePosition.Set(true); 
        //IndexerWheel.Set(ControlMode::PercentOutput, -.5);
       // Indexer.Set(ControlMode::PercentOutput, -.4);       
        }
      //Intake Off
      else 
        {
          Intake.Set(0);
          IntakePosition.Set(false);
          //IndexerWheel.Set(ControlMode::PercentOutput, 0);
          if (Manipulator.GetAButton()==0)
         { //Indexer.Set(ControlMode::PercentOutput, 0);}
        }
        frc::SmartDashboard::PutNumber("SetPoint", SetPoint);

       

  }
  }
void Robot::Indexing()
  {//talon
    //Indexer.ConfigPeakCurrentLimit(30, 40);
    //Indexer.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 30, 35, 20));
    //Indexer.EnableCurrentLimit(true);
      //Constant Turn
      // if (Manipulator.GetBButton()==1 and Indexer.GetOutputCurrent()>20)
      //   {
      //     Indexer.Set(ControlMode::PercentOutput, -1);
      //   }
      double currentThresh = 1;
      if(Manipulator.GetAButton()==0 and Driver.GetBumper(frc::XboxController::kLeftHand)==0 and Driver.GetBumper(frc::XboxController::kRightHand)==0 and Indexer.GetOutputCurrent()<currentThresh)
        {
          Indexer.Set(ControlMode::PercentOutput, 0);
        }
      else if (Manipulator.GetAButton()==0 and Driver.GetBumper(frc::XboxController::kLeftHand)==0 and Driver.GetBumper(frc::XboxController::kRightHand)==1 and Indexer.GetOutputCurrent()<currentThresh)
        {
          //Intake.Set(.75);
          Indexer.Set(ControlMode::PercentOutput, -.3);
        }
      else if (Manipulator.GetAButton()==0 and Driver.GetBumper(frc::XboxController::kLeftHand)==1 and Driver.GetBumper(frc::XboxController::kRightHand)==0 and Indexer.GetOutputCurrent()<currentThresh)
        {
         // Intake.Set(-.75);
          Indexer.Set(ControlMode::PercentOutput, -.3);
        }
      else if (Manipulator.GetAButton()==1 and Driver.GetBumper(frc::XboxController::kLeftHand)==0 and Driver.GetBumper(frc::XboxController::kRightHand)==1 and Indexer.GetOutputCurrent()<currentThresh)
        {
         // Intake.Set(.75);
          Indexer.Set(ControlMode::PercentOutput, -.3);
        }
      else if(Manipulator.GetAButton()==1 and Driver.GetBumper(frc::XboxController::kLeftHand)==0 and Driver.GetBumper(frc::XboxController::kRightHand)==0 and Indexer.GetOutputCurrent()<currentThresh)
        {
          Indexer.Set(ControlMode::PercentOutput,-1);
          KickerWheel.Set(.80);
        }
      else if(Indexer.GetOutputCurrent()>currentThresh)
        {
          AntiJamTimer.Start();
          if(AntiJamTimer.Get()>= .5)
            {
              AntiJamTimerpt2.Reset();
              Indexer.Set(ControlMode::PercentOutput, .25);
            }
          
        }
      else
        {
          if( AntiJamTimerpt2.Get()<2)
          {
            AntiJamTimerpt2.Start();
            Indexer.Set(ControlMode::PercentOutput, .25);
          }
          else{
            Indexer.Set(ControlMode::PercentOutput,0);
            KickerWheel.Set(0);
            AntiJamTimer.Stop();
            AntiJamTimer.Reset();
          }
        }

    // if(Driver.GetBumper(frc::XboxController::kRightHand)==1)
    //   {
    //     Indexer.Set(ControlMode::PercentOutput, .5);
    //     //Mechanum wheels spins backward
    //   }
    // else if(Driver.GetBumper(frc::XboxController::kRightHand)==0)
    //   {
    //     Indexer.Set(ControlMode::PercentOutput, .5);
    //     //Mechanum wheelse spins backward
    //   }
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
  //Peak Outputs
    LeftFalcon1.ConfigPeakOutputForward(.85);
    RightFalcon1.ConfigPeakOutputForward(.85);
    LeftFalcon1.ConfigPeakOutputReverse(-.85);
    RightFalcon1.ConfigPeakOutputReverse(-.85);
  //Current Limits
    LeftFalcon1.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 35, 20));   
    LeftFalcon2.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 35, 20));
    RightFalcon1.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 35, 20));
    RightFalcon2.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 35,20));
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
      double ShooterP = .00200;
      double ShooterI = 0.00000;
      double ShooterD = 0.0000000;
      double ShooterIz = 0;
      double ShooterFF = .000205;
    //Configuring PIDs
      rev::CANPIDController ShooterPIDController = Shooter1.GetPIDController();
      rev::CANPIDController Shooter2PIDController = Shooter2.GetPIDController();
      ShooterPIDController.SetP(ShooterP);
      ShooterPIDController.SetI(ShooterI);
      ShooterPIDController.SetD(ShooterD);
      ShooterPIDController.SetFF(ShooterFF);
      ShooterPIDController.SetIZone(ShooterIz);
      ShooterPIDController.SetOutputRange(-.9,9);
      Shooter2PIDController.SetP(ShooterP);
      Shooter2PIDController.SetI(ShooterI);
      Shooter2PIDController.SetD(ShooterD);
      Shooter2PIDController.SetFF(ShooterFF);
      Shooter2PIDController.SetIZone(ShooterIz);
      Shooter2PIDController.SetOutputRange(-.9,9);
    //Current Limits
      Shooter1.SetSmartCurrentLimit(25, 40, 11433);
      Shooter2.SetSmartCurrentLimit(25, 40, 11433);
      Shooter1.SetSecondaryCurrentLimit(40, 20);
      Shooter2.SetSecondaryCurrentLimit(40, 20);
    //Encoder
      rev::CANEncoder ShooterEncoder = Shooter1.GetEncoder();
      rev::CANEncoder ShooterEncoder2 = Shooter2.GetEncoder();
    //Control Mode
     // ShooterPIDController.SetReference(ShooterSpeed, rev::ControlType::kVelocity);
     // Shooter1.Set(rev::ControlType::kVelocity);
     // Shooter1.Set(ShooterSpeed);
     // Shooter2.SetInverted(true);
     // Shooter1.SetInverted(false);
     // Shooter2.Follow(Shooter1, true);
      //Shooter2.Set(Shooter1.Get());
  }
void Robot::TurretPIDsShort()
  {
    //Talon
    //Defining PID
      double TurretP = 0.065;
      double TurretI = 0;
      double TurretD = 0.68;
    //Configuring PIDs
      Turret.Config_kP(0,TurretP,10);
      Turret.Config_kI(0,TurretI,10);
      Turret.Config_kD(0,TurretD,10);
      Turret.SetNeutralMode(Brake);


  }
void Robot::TurretPIDsLong()
  {
    //Talon
    //Defining PID
      double TurretP = 0.03;
      double TurretI = 0;
      double TurretD = 0.57;
    //Configuring PIDs
      Turret.Config_kP(0,TurretP,10);
      Turret.Config_kI(0,TurretI,10);
      Turret.Config_kD(0,TurretD,10);
      Turret.SetNeutralMode(Brake);
  }
void Robot::TurretPIDsSuperShort()
  {
      //Talon
    //Defining PID
      double TurretP = 0.15;
      double TurretI = 0;
      double TurretD = 0.9;
    //Configuring PIDs
      Turret.Config_kP(0,TurretP,10);
      Turret.Config_kI(0,TurretI,10);
      Turret.Config_kD(0,TurretD,10);
      Turret.SetNeutralMode(Brake);
  }
void Robot::ClimbPIDs()
  {
    //Falcon
    //Definging PIDs
      double ClimbP = 0;
      double ClimbI = 0;
      double ClimbD = 0;
      double ClimbF = .05;
    //Current Limits
      Climber1.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 35, 20));
      Climber2.ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 35 ,20));
    //Configuring PIDs
      Climber1.Config_kP(0,ClimbP,10);
      Climber1.Config_kI(0,ClimbI,10);
      Climber1.Config_kD(0,ClimbD,10);
      Climber1.Config_kF(0,ClimbF,10);
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

    //Defining Hood Positions and Angles 
      if(HoodPosition == 1) //far shot
        {
          HoodAngle = 17;
          BallAngle = 90 - HoodAngle;
        }
      else if (HoodPosition == 2) //Bumpershot
        {
          HoodAngle = 24.5; //62.5
          BallAngle = 90 - HoodAngle;
        }

      if (HoodPosition == 1)
      {
        ShooterHood1.Set(false);
        
      }
      else if (HoodPosition ==2)
      {
        ShooterHood1.Set(true);
        
      }

      
  }
void Robot::StraightLine()
  {
    LeftFalcon1.Set(ControlMode::Position, LeftPosition+100000);
    RightFalcon1.Set(ControlMode::Position, RightPosition+100000);
  }
void Robot::Pidgeon()
  {
    if(ypr[0]>=5 and LeftFalcon1.GetSelectedSensorVelocity()>0)
    {
     LeftFalcon1.ConfigPeakOutputForward(.9);
     LeftFalcon2.ConfigPeakOutputForward(.9);
     RightFalcon1.ConfigPeakOutputForward(.8);
     RightFalcon2.ConfigPeakOutputForward(.8);
    }
    else if (ypr[0]<=-5 and LeftFalcon1.GetSelectedSensorVelocity()>0)
    {
      LeftFalcon1.ConfigPeakOutputForward(.8);
      LeftFalcon2.ConfigPeakOutputForward(.8);
      RightFalcon1.ConfigPeakOutputForward(.9);
      RightFalcon1.ConfigPeakOutputForward(.9);
    }
    else if (ypr[0]>=5 and LeftFalcon1.GetSelectedSensorVelocity()<0)
    {
      LeftFalcon1.ConfigPeakOutputReverse(-.8);
      LeftFalcon2.ConfigPeakOutputReverse(-.8);
      RightFalcon1.ConfigPeakOutputReverse(-.9);
      RightFalcon2.ConfigPeakOutputReverse(-.9);
    }
    else if (ypr[0]<=-5 and LeftFalcon1.GetSelectedSensorVelocity()<0)
    {
      LeftFalcon1.ConfigPeakOutputReverse(-.9);
      LeftFalcon2.ConfigPeakOutputReverse(-.9);
      RightFalcon1.ConfigPeakOutputReverse(-.8);
      LeftFalcon2.ConfigPeakOutputReverse(-.8);
    }
    else
    {
      LeftFalcon1.ConfigPeakOutputForward(.8);
      LeftFalcon2.ConfigPeakOutputForward(.8);
      RightFalcon1.ConfigPeakOutputForward(.8);
      RightFalcon2.ConfigPeakOutputForward(.8);
      LeftFalcon1.ConfigPeakOutputReverse(-.8);
      LeftFalcon2.ConfigPeakOutputReverse(-.8);
      RightFalcon1.ConfigPeakOutputReverse(-.8);
      RightFalcon2.ConfigPeakOutputReverse(-.8);
    }
    
  }
#ifndef RUNNING_FRC_TESTS
int main() { 
  return frc::StartRobot<Robot>(); 
  }
#endif
