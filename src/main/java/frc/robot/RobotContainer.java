// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.CoralHandler.CoralHandler;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.Processor;

public class RobotContainer {

  //private Robot robot;

  
  private final Climber climber = new Climber(Constants.Climber.MotorID);
  private final CoralHandler elevator = new CoralHandler(Constants.Elevator.Lift.MotorID,Constants.Elevator.Arm.MotorID,Constants.Elevator.Intake.MotorID,Constants.Elevator.Intake.sensorPort);
  private final Processor processor = new Processor();
  public final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(Constants.Modules.moduleArray);
  SendableChooser<Command> m_chooser = new SendableChooser<>();


  public CommandXboxController DriveController = new CommandXboxController(0);
  CommandXboxController coDriveController = new CommandXboxController(1);
  CommandXboxController ohShitController = new CommandXboxController(2);
   //Trigger xButton = DriveController.x();  
  //Trigger yButton = DriveController.y();  
  //Trigger bButton = DriveController.b();  
  //Trigger ltrigger = DriveController.leftTrigger();
  //Trigger rtrigger = DriveController.rightTrigger();
  //Trigger noteAquired = new Trigger(IntakeSubsystem.getInstance()::getHasNote);
  //Trigger dPadUp = DriveController.povUp();
  //Trigger dPadDown = DriveController.povDown();
  Trigger DriverDpadUp = DriveController.povUp();
  Trigger DriverDpadDn = DriveController.povDown();
  Trigger dpadup = coDriveController.povUp();
  Trigger dpaddn = coDriveController.povDown();
  Trigger dpadr = coDriveController.povRight();
  Trigger dpadl = coDriveController.povLeft();
  Trigger lsb = coDriveController.leftStick();
  Trigger rsb = coDriveController.rightStick();
  Trigger y = coDriveController.y();
  Trigger x = coDriveController.x();
  Trigger a = coDriveController.a();
  Trigger b = coDriveController.b();
  Trigger lb = coDriveController.leftBumper();
  Trigger rb = coDriveController.rightBumper();
  Trigger rt = coDriveController.rightTrigger();
  Trigger lt = coDriveController.leftTrigger();
  Trigger manualUp = ohShitController.leftTrigger();
  Trigger manualDn = ohShitController.rightTrigger();
  Trigger manualDpadLeft = ohShitController.povLeft();
  Trigger manualDpadRight = ohShitController.povRight();
  Trigger manualY = ohShitController.y();
  Trigger manualA = ohShitController.a();
  Trigger manualX = ohShitController.x();
  public RobotContainer() {

    //robot = new_robot;
    configureBindings();
    //m_chooser.setDefaultOption("taxi", robot.taxi);
    //m_chooser.addOption("shoot", robot.shoot);
    //m_chooser.addOption("shoot and taxi", robot.shoot.andThen(robot.taxi));
    //SmartDashboard.putData(m_chooser);
  }

  private void configureBindings() {
  //drivetrain.setDefaultCommand(new DriveFieldRelative(drivetrain, DriveController));
  
  manualUp.whileTrue(elevator.manualUp());
  manualDn.whileTrue(elevator.manualDown());
  manualDpadRight.whileTrue(elevator.manualOut());
  manualDpadLeft.whileTrue(elevator.manualIn());  
  manualY.whileTrue(elevator.manualIntake());
  manualA.whileTrue(elevator.manualShoot());
  manualX.whileTrue(elevator.manualSlowShoot());

  rt.whileTrue(elevator.intakeAndHold());
  rt.onFalse(elevator.idle());
  dpadup.onTrue(elevator.gotoL4());
  dpaddn.onTrue(elevator.gotoL1());
  dpadl.onTrue(elevator.gotoL2());
  dpadr.onTrue(elevator.gotoL3());
  lsb.onTrue(elevator.release());
  rsb.onTrue(elevator.idle());
  rb.whileTrue(processor.manualDown());
  lb.whileTrue(processor.manualUp());
  x.whileTrue(processor.manualIn());
  b.whileTrue(processor.manualOut());


  DriverDpadUp.whileTrue(climber.climbCommand());
  DriverDpadDn.whileTrue(climber.releaseCommand());

    //xButton.onTrue(new IntakeSetAng(IntakeAng.Speaker));
    //yButton.onTrue(new IntakeSetAng(IntakeAng.Amp));
    //bButton.onTrue(new IntakeSetAng(IntakeAng.Extended));

    //ltrigger.and(noteAquired.negate())
    //  .whileTrue(new IntakeSetSpeed(IntakeSpeed.In));
    //rtrigger.and(IntakeSubsystem.getInstance()::getAtSetpoint)
    //  .whileTrue(new IntakeSetSpeed(IntakeSpeed.Out));

    

    //dPadUp.onTrue(new ClimberExtend());
    //dPadDown.onTrue(new ClimberRetract());
}

  
}
