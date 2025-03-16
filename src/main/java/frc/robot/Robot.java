// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.Processor;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;                                  

  CommandXboxController driveController = new CommandXboxController(0);
  XboxController coDriverController = new XboxController(1);

  //crap code//
  //ang motor
  // RealSparkMaxIO angleMotorIo = new RealSparkMaxIO(Constants.Pconstants.AngMotorID, MotorType.kBrushless, RealSparkMaxIO.EncoderType.ENCODER_TYPE_ALTERNATE);
  Processor processor;
  // PIDController angController = new PIDController(1.5,0 ,0);
  // SlewRateLimiter angLimiter = new SlewRateLimiter(2);
  //wheel motor
  // RealSparkMaxIO intakeMotorIo = new RealSparkMaxIO(Constants.Pconstants.ShootMotorID, MotorType.kBrushless);
  boolean shooting = false;
  boolean intaking = false;
  double inSpeed = 1;
  double outSpeed = -1;
  double retractedSetpoint = 0.01318359375;
  double ampSetpoint = -0.5;
  double extendedSetpoint = -1.103515625;
  double intakerotationspeed = 0.2;
  //drive
  boolean slow = false;
  boolean stop = false;
  //shooter
  //SparkMax shooter1 = new SparkMax(Constants.Shooter.aID, MotorType.kBrushless);
  //SparkMax shooter2 = new SparkMax(Constants.Shooter.bID, MotorType.kBrushless);

  //pneumatics
  // PneumaticsControlModule PCM = new PneumaticsControlModule();
  // DoubleSolenoid solenoid;
  
  public Command taxi = Commands.runOnce(()->{
    DrivetrainSubsystem.getInstance().driveFieldRelative(
      new ChassisSpeeds(-2,0,0));}).andThen(
        Commands.waitSeconds(4)).andThen(()->{
          DrivetrainSubsystem.getInstance().driveFieldRelative(new ChassisSpeeds());});
  //public Command donothing = Commands.runOnce(()->{setShooterSpeed(0);}).andThen(Commands.waitSeconds(15));
  double translationPow = 3;
  double rotationPow = 3;
  //crap code//

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    //camera//
    CameraServer.startAutomaticCapture();

    // PCM.enableCompressorDigital();
    // solenoid = PCM.makeDoubleSolenoid(1, 0);
    // solenoid.set(Value.kReverse);

    // SparkMaxConfig config = new SparkMaxConfig();
    // config.smartCurrentLimit(40);
    // config.idleMode(IdleMode.kBrake);
    // AlternateEncoderConfig ang_alt_encoder_config = new AlternateEncoderConfig();
    // ang_alt_encoder_config.countsPerRevolution(8192);
    // config.alternateEncoder.apply(ang_alt_encoder_config);
    // angleMotorIo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // SparkMaxConfig wheelConfig = new SparkMaxConfig();
    // wheelConfig.smartCurrentLimit(40);
    // wheelConfig.idleMode(IdleMode.kCoast);
    // intakeMotorIo.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        processor = new Processor();
    //shooter1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //shooter2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // angController.setSetpoint(0.01318359375);

    // Initialize here to retrieve the details regarding the gyroscope.
    // Do not use to ensure that any changes to behavior of the subsystem are unobserved and do not
    // impact the driving and autonomous of the robot.
    DrivetrainSubsystem.getInstance().resetGyro();

    // SmartDashboard.putNumber("translationPow", translationPow);
    // SmartDashboard.putNumber("rotationPow", rotationPow);
    // SmartDashboard.putNumber("speaker shoot speed", outSpeed);
    // SmartDashboard.putNumber("speaker shoot speed", inSpeed);
    // SmartDashboard.putNumber("retracted Setpoint", retractedSetpoint);
    // SmartDashboard.putNumber("amp Setpoint", ampSetpoint);
    // SmartDashboard.putNumber("extended Setpoint", extendedSetpoint);
    // SmartDashboard.putNumber("intake rotation speed", intakerotationspeed);
    // //SmartDashboard.putNumber("gyro", DrivetrainSubsystem.getInstance().getgy())

  }

  @Override
  public void robotPeriodic() {

    // SmartDashboard.putNumber("encoder Position", angleMotorIo.getPosition());

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = null;
    m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    driveController.x().whileTrue(processor.down());
    driveController.y().whileTrue(processor.up());
  }

  @Override
  public void teleopPeriodic() {

    driveRobot();
  
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {

    translationPow = SmartDashboard.getNumber("translationPow", translationPow);
    rotationPow = SmartDashboard.getNumber("translationPow", rotationPow);
    inSpeed = SmartDashboard.getNumber("Processor Intake Speed", inSpeed);
    outSpeed = SmartDashboard.getNumber("Processor shoot speed", outSpeed);
    retractedSetpoint = SmartDashboard.getNumber("retracted Setpoint", retractedSetpoint);
    ampSetpoint = SmartDashboard.getNumber("amp Setpoint", ampSetpoint);
    extendedSetpoint = SmartDashboard.getNumber("extended Setpoint", extendedSetpoint);
    intakerotationspeed = SmartDashboard.getNumber("intake rotation speed", intakerotationspeed);
    driveRobot();

  }

  @Override
  public void testExit() {}


  public void driveRobot(){
/* 
    //climber
      if(driveController.getXButtonPressed()){
        solenoid.set(Value.kReverse);
      }else if(driveController.getAButtonPressed()){
        solenoid.set(Value.kForward);
      }
    //intake
      boolean coDriverLeftBumber = coDriverController.getLeftBumperButton();
      boolean coDriverRightBumber = coDriverController.getRightBumperButton();


      double rightTriggerAxis = coDriverController.getRightTriggerAxis();
      double leftTriggerAxis = coDriverController.getLeftTriggerAxis();

      processor.Update(coDriverRightBumber, coDriverLeftBumber,
      rightTriggerAxis, leftTriggerAxis, inSpeed, outSpeed);


    //gyro reset
      if(driveController.getStartButtonPressed()){
        DrivetrainSubsystem.getInstance().resetGyro();
      }
 */
    //drive
      double x = driveController.getLeftX(),y = driveController.getLeftY(),theta = driveController.getRightX();
      
      if(Math.hypot(x, y) < Constants.controllerDeadband){x = 0; y = 0;}
      if(Math.abs(theta) < Constants.controllerDeadband){theta = 0.0;} 

      x = (x > 0)? Math.abs(Math.pow(x,translationPow)) : -Math.abs(Math.pow(x,translationPow));
      y = (y > 0)? Math.abs(Math.pow(y,translationPow)) : -Math.abs(Math.pow(y,translationPow));
      theta = (theta>0)? Math.abs(Math.pow(theta,rotationPow)) : -Math.abs(Math.pow(theta,rotationPow));

      //slow = driveController.leftBumper();
      //if(slow){DrivetrainSubsystem.getInstance().driveFieldRelative(new ChassisSpeeds(
      //  y * Constants.attainableMaxTranslationalSpeedMPS * 0.25, 
      //  x * Constants.attainableMaxTranslationalSpeedMPS * 0.25, 
      //  theta * Constants.attainableMaxRotationalVelocityRPS * 0.25));}

      double slowModeFactor = (driveController.getLeftTriggerAxis()*3)+1;

      DrivetrainSubsystem.getInstance().driveFieldRelative(new ChassisSpeeds(
        (y * Constants.attainableMaxTranslationalSpeedMPS) / slowModeFactor, 
        (x * Constants.attainableMaxTranslationalSpeedMPS) / slowModeFactor, 
        (theta * Constants.attainableMaxRotationalVelocityRPS) / slowModeFactor)
      );
  }

}