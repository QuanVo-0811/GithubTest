/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include "ctre/Phoenix.h"

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/livewindow/LiveWindow.h>

//for the Camera 
#include "frc/smartdashboard/Smartdashboard.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

//For Compressor 
#include "frc/Compressor.h"
#include "frc/Solenoid.h"
float DriveRampup = .20;
float KpAim = -0.1f;
float KpDistance = -0.1f;
float min_command = 0.05f;

float heading_error;
float distance_error;
float steering_adjust;
float distance_adjust;
float lift;

float RobotDriveSpeed;
float RobotDriveTurn;
bool ArmDown = false;
bool ColorWheelUp = false;
bool RampUp = false;
bool ShooterOn = false;
bool shooterStarted = false;
bool polarity = false;

/* 
void HatchSubsystem::OpenHatch(){
  pSolenoid1->Set(true);
}

void HatchSubsystem::CloseHatch(){
  pSolenoid1->Set(false);
} */



class Robot : public frc::TimedRobot {

  private:
    /* 
    //Example:
    TalonSRX srx = TalonSRX(0);
    srx.setNeutralMode(NeutralMode.Break);
    //open loop basically means that there is nothing from the output giving you feed back of the out put 
    srx.ConfigOpenloopRamp(0.5); // 0.5 seconds from neutral to full output (during open-loop control)
    //close loop is that the output will give the speed controller feedback like an encoder 
    srx.configClosedloopRamp(0); // 0 disables ramping (during closed-loop control)
    */

    /*
    CAN numbers and names 
    Low Shooter Left = 40 
    Drive Motor Right Bottom = 30 
    Drive Motor Right Middle = 29
    Drive Motor Right Top = 28
    Drive Motor Left Bottom = 27 
    Drive Motor Left Middle = 26
    Drive Motor Left top = 25
    Intake Right = 24
    Intake Left = 23
    Ball Feeder Right = 21
    Shooter Right = 20
    Shooter Left = 19
    Lower Wheel = 10
    Whench Left = 9
    Whench Right = 8

    */

    //Drive Motors 
    WPI_VictorSPX m_LeftFront = WPI_VictorSPX(13); //we are skipping 0 because if we change a speed contoller we need to be able to find it 
    WPI_VictorSPX m_LeftMiddle = WPI_VictorSPX(14);
    WPI_VictorSPX m_LeftBack = WPI_VictorSPX(15);
    WPI_VictorSPX m_RightFront= WPI_VictorSPX(1);
    WPI_VictorSPX m_RightMiddle = WPI_VictorSPX(2);
    WPI_VictorSPX m_RightBack = WPI_VictorSPX(3);
    frc::DifferentialDrive RobotDrive{m_LeftFront, m_RightFront};

    //Ball intake/collector
    WPI_VictorSPX m_PowerCellIntake = WPI_VictorSPX(10);//BallPickerUpper

    //Ball motors
    WPI_VictorSPX m_BallDumper = WPI_VictorSPX(12);//BallDumper
    
    //TalonSRX m_Shooter = TalonSRX(9);
    WPI_VictorSPX m_Shooter = WPI_VictorSPX(6);
    WPI_VictorSPX m_Shooter2 = WPI_VictorSPX(9);

    WPI_VictorSPX m_Trigger = WPI_VictorSPX(7); //ShooterBallTrigger

    //Lift Motors:
    WPI_VictorSPX m_LeftLift = WPI_VictorSPX(5);
    WPI_VictorSPX m_RightLift = WPI_VictorSPX(4);
    
    //Control Panel Color Wheel Motors: 
    WPI_VictorSPX m_ControlPanel = WPI_VictorSPX(8);//ColorWheel 
/*
    //Lift Motor
    WPI_VictorSPX m_LeftLift = WPI_VictorSPX(11);
    WPI_VictorSPX m_RightLift = WPI_VictorSPX(4);
*/
    //Joystick-Controllers
    frc::Joystick JoystickMainDriver{0};
    frc::Joystick JoystickCoDriver{1};

    //Compressor 
    frc::Compressor CompressorObject  = frc::Compressor(0);
    frc::Solenoid Ramp = frc::Solenoid(0,2);
    frc::Solenoid ColorWheel = frc::Solenoid(0,1);
    frc::Solenoid Arm = frc::Solenoid(0,3);


    frc::LiveWindow& m_lw = *frc::LiveWindow::GetInstance();
    frc::Timer m_timer;
    frc::Timer t_GameTime;
    frc::Timer m_timeclock;
    frc::Timer button_timer;



    bool ManualRobotDrive = true;


  public:
  Robot() {
    
    //Reset the Speed Controllers settings 
    m_LeftFront.ConfigFactoryDefault();
    m_LeftMiddle.ConfigFactoryDefault();
    m_LeftBack.ConfigFactoryDefault();
    m_RightFront.ConfigFactoryDefault();
    m_RightMiddle.ConfigFactoryDefault();
    m_RightBack.ConfigFactoryDefault();

    //Set the other drive motors to follow the front motor 
    m_LeftMiddle.Follow(m_LeftFront);
    m_LeftBack.Follow(m_LeftFront);
    m_RightMiddle.Follow(m_RightFront);
    m_RightBack.Follow(m_RightFront);

    //Set the Drive Motors to break mode and to ramp up in one 1 second
    m_LeftFront.SetNeutralMode(NeutralMode(Brake));
    m_LeftFront.ConfigOpenloopRamp(DriveRampup);
    m_LeftFront.SetInverted(true);
    m_LeftFront.SetExpiration(0.1);
    m_LeftFront.SetSafetyEnabled(true);

    m_LeftMiddle.SetNeutralMode(NeutralMode::Brake);
    m_LeftMiddle.SetSafetyEnabled(false);
    //m_LeftMiddle.ConfigOpenloopRamp(DriveRampup);
    m_LeftMiddle.SetInverted(true);
    m_LeftBack.SetNeutralMode(NeutralMode::Brake);
    m_LeftBack.SetSafetyEnabled(false);
    //m_LeftBack.ConfigOpenloopRamp(DriveRampup);
    m_LeftBack.SetInverted(true);


    m_RightFront.SetNeutralMode(NeutralMode::Brake);
    m_RightFront.ConfigOpenloopRamp(DriveRampup);
    m_RightFront.SetInverted(true);
    m_RightFront.SetExpiration(0.1);
    m_RightFront.SetSafetyEnabled(true);

    m_RightMiddle.SetNeutralMode(NeutralMode::Brake);
    //m_RightMiddle.ConfigOpenloopRamp(DriveRampup);
    m_RightMiddle.SetInverted(true);
    m_RightMiddle.SetSafetyEnabled(false);
    m_RightBack.SetNeutralMode(NeutralMode::Brake);
    //m_RightBack.ConfigOpenloopRamp(DriveRampup);
    m_RightBack.SetInverted(true);
    m_RightBack.SetSafetyEnabled(false);

  
    m_Shooter.SetInverted(true); //Invert the Shooter motor 
    m_Shooter.ConfigOpenloopRamp(.25); //Ramp up the motor 1/4 second
    m_Shooter2.Follow(m_Shooter); //Set second Shooter motor to do the same as the main motor

    //Set the Ball Shooter Feeder/Trigger inverted, with breake mode and ramp up of 1/4 seconds
    m_Trigger.SetInverted(true);
    m_LeftFront.SetNeutralMode(NeutralMode(2));
    m_LeftFront.ConfigOpenloopRamp(.25);

    //Compressor 
    CompressorObject.SetClosedLoopControl(true);

    //Lift 
    m_LeftLift.SetInverted(true);
    m_LeftLift.Follow(m_RightLift);

    RobotDrive.SetExpiration(0.1);
    m_timer.Start();
  }

  void AutonomousInit() override {
      std::cout << "Auto selected: " << "test" << std::endl;

    m_timer.Reset();
    m_timer.Start();

    //forgot to start the timer -PG
    t_GameTime.Reset();
    t_GameTime.Start();
  }

  void AutonomousPeriodic() override {
      // Drive moveForward
      if (t_GameTime.Get() < 2.5) {
        // Drive forwards half speed
        RobotDrive.ArcadeDrive(0.5, 0.0);
      } else {
        // Stop robot
        RobotDrive.ArcadeDrive(0.0, 0.0);
      }

      if(t_GameTime.Get() >= 0.5 && t_GameTime.Get() <= 6){
        Ramp.Set(true);
      }
      else
      {
        Ramp.Set(false);
      }
      

      // Shooter
      if((t_GameTime.Get() >= 1) && (t_GameTime.Get() <= 7)){
        m_Shooter.Set(.6);
        //std::cout << "Shooter: " << "ON" << std::endl;

      } 
      else{
        m_Shooter.Set(0);
        //std::cout << "Shooter: " << "OFF" << std::endl;
      }
       // Feeder run
      if((t_GameTime.Get() >= 4)  && t_GameTime.Get() <= 6){
        m_Trigger.Set(.6);
      }
      else {
        // Stop Feeder
        m_Trigger.Set(0);
      } 
      
  
    /*    
      // Limelight for auton 
      if((t_GameTime.Get() >= 7) && !(shooterStarted)) {
        m_timeclock.Start();
        shooterStarted = true;
      }
      if (m_timeclock.Get() <= 3.0) {
        std::shared_ptr<NetworkTable> LimeLightNetTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");    
        double targetOffsetAngle_Horizontal = LimeLightNetTable->GetNumber("tx",0.0);
        double targetOffsetAngle_Vertical = LimeLightNetTable->GetNumber("ty",0.0);

        LimeLightNetTable->PutNumber("ledMode",3);

        heading_error = -targetOffsetAngle_Horizontal;
        distance_error = -targetOffsetAngle_Vertical;
        steering_adjust = 0.0f;
        
        if (targetOffsetAngle_Horizontal > 1.0){
          steering_adjust = KpAim*heading_error - min_command;
        }
        else if (targetOffsetAngle_Horizontal < 1.0){
          steering_adjust = KpAim*heading_error + min_command;
        }
        distance_adjust = KpDistance * distance_error;

        RobotDrive.ArcadeDrive(distance_adjust,steering_adjust);
        

        if((distance_adjust = 0) && (steering_adjust = 0) ){
          //led off 
          LimeLightNetTable->PutNumber("ledMode",1);
        
      */
          
  }


  void TeleopInit() override {
    //Setting Pneumatic, must be configured
  /*  ArmDown = false;
    RampUp = true; */
  }

    
  void TeleopPeriodic() override {
  
    // Main-Driver Function
    // Arcard Drive
  if(ManualRobotDrive){
      RobotDrive.ArcadeDrive(RobotDriveSpeed, RobotDriveTurn);
      //RobotDrive.TankDrive(RobotDriveSpeed, RobotDriveTurn);
    }       
    
    if (JoystickMainDriver.GetRawButtonReleased(1)){
      if(polarity){
        polarity = false;
      }
      else{
        polarity = true;
      }
    }

    if(polarity){
      RobotDriveSpeed = JoystickMainDriver.GetY();
      RobotDriveTurn = JoystickMainDriver.GetZ();    }
    else{
      RobotDriveSpeed = -JoystickMainDriver.GetY();
      RobotDriveTurn = JoystickMainDriver.GetZ();  
    }

    
    //Robot Drive Half Speed
    if (JoystickMainDriver.GetRawButton(8)){ //LT
      RobotDriveSpeed *=  0.5;
      RobotDriveTurn *=  0.5;
    }

    // Trigger 
    // Forward
    if(JoystickMainDriver.GetRawButton(6)){ //Main RB
      m_Trigger.Set(1);
    }
    else if(JoystickMainDriver.GetRawButton(2)){ //Main LB
      m_Trigger.Set(-1);
    }
    else if(JoystickCoDriver.GetRawButton(8)){
      m_Trigger.Set(.5);
    }
    else {
      m_Trigger.Set(0);
    }

    //Intake Main Driver 
    if(JoystickMainDriver.GetRawButton(5)){ 
      m_PowerCellIntake.Set(1);
    } 
    else if (JoystickMainDriver.GetRawButton(7)){
      m_PowerCellIntake.Set(-1);  
    }
    else if(JoystickCoDriver.GetRawButton(7)){ 
      m_PowerCellIntake.Set(.5);
    } 
    else {
      m_PowerCellIntake.Set(0);
    }


    // Co-Driver Function
    // Pneumatic Arm 
    if(JoystickCoDriver.GetRawButtonReleased(2)){
      if(ArmDown){
        ArmDown = false;
        RampUp = true;
      }  
      else {
        ArmDown = true;
        RampUp = false;
      }
    }

    // Solonoid Arm Code
    if(ArmDown) { 
      Arm.Set(true); 
    }
    else{
      Arm.Set(false);
    }

    // Pneumatic Ramp
    if(JoystickCoDriver.GetRawButtonReleased(5)){
      if(RampUp){
        RampUp = false;
      }
      else{
        RampUp = true;
      }
    }

    // Solonoid Ramp Code
    if (RampUp) {
      Ramp.Set(true);
    } 
    else {
      Ramp.Set(false);
    }

  // Pneumatic ColorWheel
    if(JoystickCoDriver.GetRawButtonReleased(1)){
      if(ColorWheelUp){
        ColorWheelUp = false;
      }
      else
      {
        ColorWheelUp = true;
      }
    }

    // Solonoid ColorWheel Code
    if (ColorWheelUp) {
      ColorWheel.Set(true);
    } 
    else {
      ColorWheel.Set(false);
    }

   // Control Panel (motor)
    m_ControlPanel.Set((JoystickCoDriver.GetX())*-1);

   // One Way Lift
/*    if(JoystickCodriver.GetTwist() > 0){
     lift = JoystickCodriver.GetTwist();
   } if (JoystickCodriver.GetTwist() < 0 && JoystickCodriver.GetRawButton(9)) {
     lift = JoystickCoDriver.GetTwist();
   }
   m_RightLift.Set(lift);  */
   
    

    //One Way Lift 
    /*if(JoystickCoDriver.GetY() < 0){
      lift = JoystickCoDriver.GetY() * -1;
    }else{
      lift = JoystickCoDriver.GetY();
    }
    
    m_RightLift.Set(lift); */
    m_RightLift.Set(JoystickCoDriver.GetThrottle()); 

    //m_RightLift.Set((JoystickCoDriver.GetThrottle() * 0.5)); 
    /* if (JoystickCoDriver.GetRawButton(9)){ //SELECT
    m_RightLift.Set((JoystickCoDriver.GetThrottle() * 0.5)); 
    }
    else{
      m_RightLift.Set(JoystickCoDriver.GetThrottle()); 
    } */

/* 
  //Lift (practice) TEST
    //Base on the research this should work for the d-pad 0=up, 180=down
    if(JoystickCoDriver.GetPOV(0)){ 
      m_RightLift.Set(.1);
    } 
    else if(JoystickCoDriver.GetPOV(180)){ 
      m_RightLift.Set(-.1);
    } 
    else {
      m_RightLift.Set(0);
    } */


    //Toggle for co-driver=
 /*    if(JoystickCoDriver.GetRawButtonReleased(6)){ //CO RB
      if(ShooterOn){
        ShooterOn = false;
      }
      else{
        ShooterOn = true;
      } */
      


    // Shooter On Function
    if(JoystickCoDriver.GetRawButton(6)){
      m_Shooter.Set(.6);
    }
    else{
      m_Shooter.Set(0);
    }

    if(JoystickMainDriver.GetRawButton(6)){ //Main RB
      m_Trigger.Set(1);
    }
    else if(JoystickMainDriver.GetRawButton(2)){ //Main A
    //reverse
      m_Trigger.Set(-1);
    }
    else {
      m_Trigger.Set(0);
    }

    //Intake
    if(JoystickMainDriver.GetRawButton(5)){ //Main LB
      m_PowerCellIntake.Set(1);
    } 
    else if (JoystickMainDriver.GetRawButton(7)){ // Main LT
      m_PowerCellIntake.Set(-1);  
    }
    else {
      m_PowerCellIntake.Set(0);
    }


    //Camera test code 
    /*Camera Controls:
    http://docs.limelightvision.io/en/latest/networktables_api.html
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("<variablename>",<value>);
    ledMode 	Sets limelight’s LED state
          0 	use the LED Mode set in the current pipeline
          1 	force off
          2 	force blink
          3 	force on
    */

    std::shared_ptr<NetworkTable> LimeLightNetTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");    
    double targetOffsetAngle_Horizontal = LimeLightNetTable->GetNumber("tx",0.0);
    double targetOffsetAngle_Vertical = LimeLightNetTable->GetNumber("ty",0.0);
    //double targetArea = LimeLightNetTable->GetNumber("ta",0.0);
    //double targetSkew = LimeLightNetTable->GetNumber("ts",0.0);

    /*if (JoystickMainDriver.GetRawButton(1)){
      LimeLightNetTable->PutNumber("ledMode",3);
    }
    else{
      LimeLightNetTable->PutNumber("ledMode",1);
    } */

    //Camare seeking and aiming. 
    if (JoystickMainDriver.GetRawButton(4)){ //Main Y 
      //led on   
      LimeLightNetTable->PutNumber("ledMode",3);

      heading_error = -targetOffsetAngle_Horizontal;
      distance_error = -targetOffsetAngle_Vertical;
      steering_adjust = 0.0f;
      
      if (targetOffsetAngle_Horizontal > 1.0){
        steering_adjust = KpAim*heading_error - min_command;
      }
      else if (targetOffsetAngle_Horizontal < 1.0){
        steering_adjust = KpAim*heading_error + min_command;
      }
      distance_adjust = KpDistance * distance_error;
      distance_adjust = distance_adjust * -1;

      ManualRobotDrive = false;
      RobotDrive.ArcadeDrive(distance_adjust,steering_adjust);
    }
    else if(JoystickMainDriver.GetRawButton(3)){
      LimeLightNetTable->PutNumber("ledMode",3);
    }
    else{
      //led off 
      LimeLightNetTable->PutNumber("ledMode",1);
      ManualRobotDrive = true;  
    }
  }
  
  void TestPeriodic() override {}
};
#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
