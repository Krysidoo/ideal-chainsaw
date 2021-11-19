// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//CODE WRITTED BY RAYAN.B
#include "Robot.h"

#include <iostream>

#include <frc/smartdashboard/SmartDashboard.h>
//////////////////////////////////////////////////////////////////////////
void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom1, kAutoNameCustom1);
  m_chooser.AddOption(kAutoNameCustom2, kAutoNameCustom2);
  m_chooser.AddOption(kAutoNameCustom3, kAutoNameCustom3);
  m_chooser.AddOption(kAutoNameCustom4, kAutoNameCustom4);
  m_chooser.AddOption(kAutoNameCustom5, kAutoNameCustom5);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  

  left_encoder.SetDistancePerPulse( (3.14159 * 6) / 256);

  try
  {
    ahrs = new AHRS(SPI::Port::kMXP);
    //ahrs = new AHRS(SerialPort::Port::kUSB1);
  }
  catch(const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  
}

/////////////////////////////////////////////////////////////////////////
void Robot::RobotPeriodic() {
/*
frc::SmartDashboard::PutNumber("ia: ", ia); 
frc::SmartDashboard::PutNumber("it: ", it); 

frc::SmartDashboard::PutBoolean("LB : ", leftbump); 
frc::SmartDashboard::PutBoolean("RB : ", rightbump);

frc::SmartDashboard::PutBoolean("bA :", bA);
frc::SmartDashboard::PutBoolean("bB :", bB);
frc::SmartDashboard::PutBoolean("bX :", bX);
frc::SmartDashboard::PutBoolean("bY :", bY);

frc::SmartDashboard::PutNumber("left x : ", left_x);
frc::SmartDashboard::PutNumber("left y : ", left_x);
frc::SmartDashboard::PutNumber("right x : ", right_x);
frc::SmartDashboard::PutNumber("right y : ", right_y);


frc::SmartDashboard::PutBoolean("limit switch : ", limit_switch_value);

frc::SmartDashboard::PutNumber("ai raw : ", ai_raw);
frc::SmartDashboard::PutNumber("ai voltage : ", ai_voltage);

frc::SmartDashboard::PutNumber("ultra raw : ", ultra_raw);
frc::SmartDashboard::PutNumber("ultra distance : ", distance);

frc::SmartDashboard::PutNumber("enc count : " , encoder_count);
frc::SmartDashboard::PutNumber("enc dist : " , encoder_distance);

frc::SmartDashboard::PutBoolean("comp sw : ", compressor_status);
*/

frc::SmartDashboard::PutNumber("Yaw : ", yaw );
frc::SmartDashboard::PutNumber("left side : ", left_side);
frc::SmartDashboard::PutNumber("right side : ", right_side);
frc::SmartDashboard::PutNumber("pid output : ", pid_output);
frc::SmartDashboard::PutNumber("elev pos : ", elevator_position);
}

////////////////////////////////////////////////////////////////////////
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom1) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

 ia =0;
 
 left_encoder.Reset();
 
}
//////////////////////////////////////////////////////////
void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom1) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

  
  
   ia++;
   //new
   encoder_count = left_encoder.Get();
   encoder_distance = left_encoder.GetDistance();
}
/////////////////////////////////////////////////////////
void Robot::TeleopInit() {
  it = 0;
  
  m_yaw_pidController.Reset();
  m_yaw_pidController.EnableContinuousInput(-180, 180);
  m_yaw_pidController.SetSetPoint(0.0);
////////////////////////////////////////////////////////
elevator.ConfigFactoryDefault();

elevator.ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 50);

elevator.SetSensorPhase(false);
elevator.SetInverted(false);

elevator.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
elevator.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);

elevator.ConfigNominalOutputForward(0, 10);
elevator.ConfigNominalOutputForward(0, 10);
elevator.ConfigPeakOutputReverse(1, 10);
elevator.ConfigPeakOutputReverse(-1, 10);

elevator.SelectProfileSlot(0, 0);
elevator.Config_kF(0, 0.3, 10);
elevator.Config_kP(0, 25.0, 10);
elevator.Config_kI(0, -50.0, 10);
elevator.Config_kD(0, -10.0, 10);

elevator.ConfigMotionCruiseVelocity(1500, 10);
elevator.ConfigMotionAcceleration(1500, 10);

elevator.SetSelectedSensorPosition(0, 0, 10);
///////////////////////////////////////////////////
}


/////////////////////////////////////////////////////
void Robot::TeleopPeriodic() { 
  it++;

  leftbump = xbox.GetBumper(frc::GenericHID::kLeftHand);
  rightbump = xbox.GetBumper(frc::GenericHID::kRightHand);

  bA = xbox.GetAButton();
  bB = xbox.GetBButton();
  bX = xbox.GetXButton();
  bY = xbox.GetYButton();
  
  left_x = xbox.GetX(frc::GenericHID::kLeftHand);
  left_y = xbox.GetY(frc::GenericHID::kLeftHand);

  right_x = xbox.GetX(frc::GenericHID::krightHand);
  right_y = xbox.GetY(frc::GenericHID::KrightHand);
 
 left_trigger = xbox.GetRawAxis(2);
 right_trigger = xbox.GetRawAxis(3);
 
 
 
  
  limit_switch_value = limit_switch.Get();
  
  ai_raw = ai.GetValue();
  ai_voltage = ai.GetVoltage();

  ultra_raw = ultra.GetValue();
  distance = (double) ultra_raw * adc_to_mm ;

  yaw = ahrs->GetYaw();
  elevator_position = elevator.GetSelectedSensorPosition(0);
  
 ///////////////////////////////////////////////////////////

 //think
/*
 pid_output = m_yaw_pidController.Calculate(yaw);
 pid_output = std::clamp(pid+output, -1.0, 1.0);

 m_robotDrive.ArcadeDrive(0, -pid_output);  // opposé ou inversé
 */
 left_side = left_motor1.Get();
 right_side = right_motor1.Get();

 /////////////////////////////////////////////////////////////

 s1.Set(0.0);
 s2.Set(0.0); 

 // 2 motor drivetrain
 left_y = std::clamp( left_y, -0.5, 0.5);
 right_y = std::clamp( right_y, -0.5, 0.5);
 m_robotDrive.SetDeadband(0.1);
 m_robotDrive.TankDrive(left_y, right_y, true);

 if(bY)
   m_yaw_pidController.SetSetpoint(0.0);

  if(bB)
   m_ m_yaw_pidController.SetSetpoint(90.0);

  if(bX)
   m_yaw_pidController.SetSetpoint(-90.0);
  
  if(bA)
   m_yaw_pidController.SetSetpoint(179.99);
  
  if (bA)
   elevator.Set(ControlMode::Position, 0);
 if (bX)
   elevator.Set(ControlMode::Position, 1000);
 if (bB)
   elevator.Set(ControlMode::Position, 1500);
 if (bY)
   elevator.Set(ControlMode::Position, 2000);

 intake.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, left_trigger);
 //elevator.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, right_trigger);

 compressor_status = compressor.GetPressureSwitchValue();

 solenoid0.Set(bA);
 solenoid1.Set(bB);
 solenoid2.Set(bX);
 solenoid3.Set(bY);
}
////////////////////////////////////////////////////////////////////////
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
