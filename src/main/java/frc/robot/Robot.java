// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import frc.robot.Subsystems.DrivetrainSubsystem;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;     
  
  //private final Climber climber = new Climber(Constants.Climber.MotorID);
  //private final Elevator elevator = new Elevator(Constants.Elevator.Lift.MotorID,Constants.Elevator.Arm.MotorID,Constants.Elevator.Intake.MotorID,Constants.Elevator.Intake.sensorPort);

  XboxController driveController = new XboxController(0);
  //CommandXboxController coDriverController = new CommandXboxController(1);

  //Processor processor;
  double translationPow = Constants.Drivetrain.TranslationPow;
  double rotationPow = Constants.Drivetrain.RotationPow;
  
  public Command taxi = Commands.deadline(
      waitSeconds(3),
      Commands.run(() -> { DrivetrainSubsystem.getInstance().drive(new ChassisSpeeds(-2.5, 0, 0)); })
    )
    .andThen(() -> { DrivetrainSubsystem.getInstance().drive(new ChassisSpeeds(0, 0, 0)); });

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    //CameraServer.startAutomaticCapture();

    //processor = new Processor();
    
    // Initialize here to retrieve the details regarding the gyroscope.
    // Do not use to ensure that any changes to behavior of the subsystem are unobserved and do not
    // impact the driving and autonomous of the robot.
    DrivetrainSubsystem.getInstance().resetGyro();

  }

  @Override
  public void robotPeriodic() {
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
    m_autonomousCommand = taxi;
    // m_robotContainer.getAutonomousCommand();

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
    //coDriverController.povUp().onTrue(elevator.gotoL4());
    //coDriverController.povDown().onTrue(elevator.gotoL1());
    //coDriverController.povLeft().onTrue(elevator.gotoL2());
    //coDriverController.povRight().onTrue(elevator.gotoL4());

    //coDriverController.povUp().whileTrue(elevator.manualUp());
    //coDriverController.povLeft().whileTrue(elevator.manualDown());
    //coDriverController.povUp().whileTrue(elevator.manualIn());
    //coDriverController.povRight().whileTrue(elevator.manualOut());  
    //coDriverController.leftStick().whileTrue(elevator.manualIntake());
    //coDriverController.rightStick().whileTrue(elevator.manualShoot()); 
//
//
    //coDriverController.y().onTrue(processor.autointake());
    //coDriverController.a().onTrue(processor.autoshoot());
    //coDriverController.x().onTrue(processor.store());
    //coDriverController.leftBumper().whileTrue(climber.climbCommand());
    //coDriverController.rightBumper().whileTrue(climber.releaseCommand());
    
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
    driveRobot();
  }

  @Override
  public void testExit() {}


  public void driveRobot() {
    if(driveController.getStartButtonPressed()){
      DrivetrainSubsystem.getInstance().resetGyro();
    }


    double x = driveController.getLeftX(), y = driveController.getLeftY(), theta = driveController.getRightX();
    
    if (Math.hypot(x, y) < Constants.controllerDeadband) {
      x = 0;
      y = 0;
    }
    if (Math.abs(theta) < Constants.controllerDeadband) {
      theta = 0.0;
    }

    x = (x > 0) ? Math.abs(Math.pow(x, translationPow)) : -Math.abs(Math.pow(x, translationPow));
    y = (y > 0) ? Math.abs(Math.pow(y, translationPow)) : -Math.abs(Math.pow(y, translationPow));
    theta = (theta > 0) ? Math.abs(Math.pow(theta, rotationPow)) : -Math.abs(Math.pow(theta, rotationPow));

    double slowModeFactor = (driveController.getLeftTriggerAxis() * Constants.Drivetrain.SlowFactor) + Constants.Drivetrain.SlowFactorOffset;

    DrivetrainSubsystem.getInstance().driveFieldRelative(
      new ChassisSpeeds(
        (y * Constants.attainableMaxTranslationalSpeedMPS) / slowModeFactor, 
        (x * Constants.attainableMaxTranslationalSpeedMPS) / slowModeFactor, 
        -(theta * Constants.attainableMaxRotationalVelocityRPS) / slowModeFactor
      )
    );
  }

}