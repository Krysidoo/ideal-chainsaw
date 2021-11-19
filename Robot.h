// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//CODE WRITTED BY RAYAN.B
#pragma once

#include <string>
#include "AHRS.h"
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/XboxController.h>
#include <frc/DigitalInput.h>
#include <frc/AnalogInput.h>
#include <frc/Servo.h>
#include <frc/VictorSP.h>
#include <frc/Encoder.h>
#include <frc/Compressor.h>
#include <frc/Solenoid.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/controller/PIDController.h>
#include <ctre/Phoenix.h>

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void TestInit() override;
  void TestPeriodic() override;

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom1 = "My Auto";
  const std::string kAutoNameCustom2 = "My Auto2";
  const std::string kAutoNameCustom3 = "My Auto3";
  const std::string kAutoNameCustom4 = "My Auto4";
  const std::string kAutoNameCustom5 = "My Auto5";
  std::string m_autoSelected;

  double kP = 0.05;
  double kI = 0.0;
  double KD = 0.0;

  int it = 0; 
  int ia = 0;

 frc::XboxController xbox{0};
 frc::DigitalInput limit_switch{9};
 frc::AnalogInput  ai{0};
 frc::AnalogInput  ultra{1};
 frc::Encoder      left_encoder{0,1};
 
 //new(may only have) PMW
 frc::Servo        s1 {8};
 frc::Servo        s2 {9};
 frc::VictorSP        left_motor1{0};
 frc::VictorSP        left_motor2{1};
 frc::VictorSP        right_motor1{2};
 frc::VictorSP        right_motor2{3};

 //can
 VictorSPX intake {0};
 TalonSRX elevator {0};



//Pneumatique
frc::Compressor compressor {0};
frc::Solenoid solenoid0 {0};
frc::Solenoid solenoid1 {1};
frc::Solenoid solenoid2 {2};
frc::Solenoid solenoid3 {3};

frc::SpeedControllerGroup m_left{left_motor1, left_motor2};
frc::SpeedControllerGroup m_right{right_motor1, right_motor2};
frc::DifferentialDrive m_robotDrive {m_left, m_right};

 frc2::PIDController m_yaw_pidController { kP, kI, kD};
 //B(lettre(ABXY)) signifie Button A,B,X,Y

 bool leftbump = false;
 bool rightbump = false;
 bool bA = false;
 bool bB = false;
 bool bX = false;
 bool bY = false;


 double left_x = 0.0;
 double left_y = 0.0;
 double right_x = 0.0;
 double right_y = 0.0;

 //new
 double left_trigger = 0.0;
 double right_trigger = 0.0;
//new
bool limit_switch_value = false;
//
int ai_raw = 0;
double ai_voltage = 0.0;

int ultra_raw = 0;
double distance = 0.0;
double adc_to_mm = 1.25 / 1000.0;
//new
int encoder_count = 0;
double encoder_distance = 0.0;

bool compressor_status;

double speed = 0.0;
double rotation = 0.0;

double sign = 1.0;

AHRS *ahrs;
float yaw;

double pid_output = 0.0;
double left_side = 0.0;
double right_side = 0.0;

int elevator_position;
};
