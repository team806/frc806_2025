// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.lang.model.util.ElementScanner14;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.DrivetrainSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import frc.robot.Subsystems.Intake;
import frc.robot.IO.RealSparkMaxIO;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;                                  

  CommandXboxController driveController = new CommandXboxController(0);
  XboxController coDriverController = new XboxController(1);

  //crap code//
  //ang motor
  RealSparkMaxIO angleMotorIo = new RealSparkMaxIO(Constants.Intake.AngMotorID, MotorType.kBrushless, RealSparkMaxIO.EncoderType.ENCODER_TYPE_ALTERNATE);
  Intake intake;
  PIDController angController = new PIDController(1.5,0 ,0);
  SlewRateLimiter angLimiter = new SlewRateLimiter(2);
  //wheel motor
  RealSparkMaxIO intakeMotorIo = new RealSparkMaxIO(Constants.Intake.ShootMotorID, MotorType.kBrushless);
  boolean shooting = false;
  boolean intaking = false;
  double ampShootSpeed = -0.7;
  double speakerShootSpeed = -1;
  double retractedSetpoint = 0.01318359375;
  double ampSetpoint = -0.5;
  double extendedSetpoint = -1.103515625;
  double intakerotationspeed = 0.2;
  //drive
  boolean slow = false;
  boolean stop = false;
  //shooter
  SparkMax shooter1 = new SparkMax(Constants.Shooter.aID, MotorType.kBrushless);
  SparkMax shooter2 = new SparkMax(Constants.Shooter.bID, MotorType.kBrushless);
  public void setShooterSpeed(double speed){shooter1.set(-speed);shooter2.set(speed);}
  //pneumatics
  PneumaticsControlModule PCM = new PneumaticsControlModule();
  DoubleSolenoid solenoid;
  
  public Command shoot = Commands.runOnce(()->{
    setShooterSpeed(1.0);}).andThen(
      Commands.waitSeconds(1)).andThen(
        Commands.runOnce(()->{intakeMotorIo.setSpeed(-1);})).andThen(
          Commands.waitSeconds(1)).andThen(
            Commands.runOnce(()->{intakeMotorIo.setSpeed(0);setShooterSpeed(0);}));
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

    PCM.enableCompressorDigital();
    solenoid = PCM.makeDoubleSolenoid(1, 0);
    solenoid.set(Value.kReverse);

    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit(40);
    config.idleMode(IdleMode.kBrake);
    AlternateEncoderConfig ang_alt_encoder_config = new AlternateEncoderConfig();
    ang_alt_encoder_config.countsPerRevolution(8192);
    config.alternateEncoder.apply(ang_alt_encoder_config);
    angleMotorIo.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    SparkMaxConfig wheelConfig = new SparkMaxConfig();
    wheelConfig.smartCurrentLimit(40);
    wheelConfig.idleMode(IdleMode.kCoast);
    intakeMotorIo.configure(wheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intake = new Intake(angleMotorIo, intakeMotorIo);
    shooter1.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shooter2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    angController.setSetpoint(0.01318359375);

    // Initialize here to retrieve the details regarding the gyroscope.
    // Do not use to ensure that any changes to behavior of the subsystem are unobserved and do not
    // impact the driving and autonomous of the robot.
    DrivetrainSubsystem.getInstance().resetGyro();

    SmartDashboard.putNumber("translationPow", translationPow);
    SmartDashboard.putNumber("rotationPow", rotationPow);
    SmartDashboard.putNumber("amp shoot speed", ampShootSpeed);
    SmartDashboard.putNumber("speaker shoot speed", speakerShootSpeed);
    SmartDashboard.putNumber("retracted Setpoint", retractedSetpoint);
    SmartDashboard.putNumber("amp Setpoint", ampSetpoint);
    SmartDashboard.putNumber("extended Setpoint", extendedSetpoint);
    SmartDashboard.putNumber("intake rotation speed", intakerotationspeed);
    //SmartDashboard.putNumber("gyro", DrivetrainSubsystem.getInstance().getgy())

  }

  @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber("encoder Position", angleMotorIo.getPosition());

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
    // m_autonomousCommand = shoot;
    // m_robotContainer.getAutonomousCommand();

    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    // }
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
    ampShootSpeed = SmartDashboard.getNumber("amp shoot speed", ampShootSpeed);
    speakerShootSpeed = SmartDashboard.getNumber("speaker shoot speed", speakerShootSpeed);
    SmartDashboard.getNumber("", ampShootSpeed);
    retractedSetpoint = SmartDashboard.getNumber("retracted Setpoint", retractedSetpoint);
    ampSetpoint = SmartDashboard.getNumber("amp Setpoint", ampSetpoint);
    extendedSetpoint = SmartDashboard.getNumber("extended Setpoint", extendedSetpoint);
    intakerotationspeed = SmartDashboard.getNumber("intake rotation speed", intakerotationspeed);
    driveRobot();

  }

  @Override
  public void testExit() {}


  public void driveRobot(){

    //climber
      // if(driveController.getXButtonPressed()){
      //   solenoid.set(Value.kReverse);
      // }else if(driveController.getAButtonPressed()){
      //   solenoid.set(Value.kForward);
      // }
    //intake
      // boolean coDriverLeftBumber = coDriverController.getLeftBumperButton();
      // boolean coDriverRightBumber = coDriverController.getRightBumperButton();

      // double encoderAng = angleMotorIo.getPosition();
      // boolean retractButtonPressed = coDriverController.getAButtonPressed();
      // boolean ampButtonPressed = coDriverController.getLeftStickButtonPressed();
      // boolean extendedButtonPressed = coDriverController.getXButtonPressed();
      // double rightTriggerAxis = coDriverController.getRightTriggerAxis();
      // double leftTriggerAxis = coDriverController.getLeftTriggerAxis();

      // intake.Update(retractButtonPressed, ampButtonPressed, extendedButtonPressed, coDriverRightBumber, coDriverLeftBumber,
      // rightTriggerAxis, leftTriggerAxis, ampShootSpeed, speakerShootSpeed);

    //shooter
      // if(coDriverController.getYButtonPressed()){
      //   shooting=!shooting;
      //   intaking=false;
      // }    
      // if(coDriverController.getBButtonPressed()){
      //   intaking=!intaking;
      //   shooting=false;
      // }
      // setShooterSpeed(shooting?1:intaking?-0.2 :0.3);
      // coDriverController.setRumble(RumbleType.kBothRumble, shooting?1:intaking?0.05:0);

    //gyro reset
      // if(driveController.getStartButtonPressed()){
      //   DrivetrainSubsystem.getInstance().resetGyro();
      // }

    //drive
      // double x = driveController.getLeftX(),y = driveController.getLeftY(),theta = driveController.getRightX();
      
      // if(Math.hypot(x, y) < Constants.controllerDeadband){x = 0; y = 0;}
      // if(Math.abs(theta) < Constants.controllerDeadband){theta = 0.0;} 

      // x = (x > 0)? Math.abs(Math.pow(x,translationPow)) : -Math.abs(Math.pow(x,translationPow));
      // y = (y > 0)? Math.abs(Math.pow(y,translationPow)) : -Math.abs(Math.pow(y,translationPow));
      // theta = (theta>0)? Math.abs(Math.pow(theta,rotationPow)) : -Math.abs(Math.pow(theta,rotationPow));

      // slow = driveController.getLeftBumperButton();
      //if(slow){DrivetrainSubsystem.getInstance().driveFieldRelative(new ChassisSpeeds(
      //  y * Constants.attainableMaxTranslationalSpeedMPS * 0.25, 
      //  x * Constants.attainableMaxTranslationalSpeedMPS * 0.25, 
      //  theta * Constants.attainableMaxRotationalVelocityRPS * 0.25));}

      // double slowModeFactor = (driveController.getLeftTriggerAxis()*3)+1;

      // DrivetrainSubsystem.getInstance().driveFieldRelative(new ChassisSpeeds(
      //   (y * Constants.attainableMaxTranslationalSpeedMPS) / slowModeFactor, 
      //   (x * Constants.attainableMaxTranslationalSpeedMPS) / slowModeFactor, 
      //   (theta * Constants.attainableMaxRotationalVelocityRPS) / slowModeFactor)
      // );

      // driveController.x().whileTrue(DrivetrainSubsystem.getInstance().executeAimCommand());
      driveController.x().whileTrue(DrivetrainSubsystem.getInstance().driveForward());
  }

}