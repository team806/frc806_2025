// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.DriveFieldRelative;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.CoralHandler.CoralHandler;
import frc.robot.Subsystems.Processor;

import frc.robot.Commands.DriveFieldRelative;

public class RobotContainer {

  //private Robot robot;

  
  private final Climber climber = new Climber(Constants.Climber.MotorID);
  private final CoralHandler elevator = new CoralHandler(Constants.Elevator.Lift.MotorID,Constants.Elevator.Arm.MotorID,Constants.Elevator.Intake.MotorID,Constants.Elevator.Intake.sensorPort);
  private final Processor processor = new Processor();
  private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(Constants.Modules.moduleArray);
  SendableChooser<Command> m_chooser = new SendableChooser<>();


  CommandXboxController DriveController = new CommandXboxController(0);
  CommandXboxController coDriveController = new CommandXboxController(1);
  //Trigger xButton = DriveController.x();  
  //Trigger yButton = DriveController.y();  
  //Trigger bButton = DriveController.b();  
  //Trigger ltrigger = DriveController.leftTrigger();
  //Trigger rtrigger = DriveController.rightTrigger();
  //Trigger noteAquired = new Trigger(IntakeSubsystem.getInstance()::getHasNote);
  //Trigger dPadUp = DriveController.povUp();
  //Trigger dPadDown = DriveController.povDown();
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

  public RobotContainer() {

    //robot = new_robot;
    configureBindings();
    //m_chooser.setDefaultOption("taxi", robot.taxi);
    //m_chooser.addOption("shoot", robot.shoot);
    //m_chooser.addOption("shoot and taxi", robot.shoot.andThen(robot.taxi));
    //SmartDashboard.putData(m_chooser);
  }

  private void configureBindings() {
  drivetrain.setDefaultCommand(new DriveFieldRelative(drivetrain, DriveController));

  /*
   * dpadup.onTrue(elevator.gotoL4());
   * dpaddn.onTrue(elevator.gotoL1());
   * dpadl.onTrue(elevator.gotoL2());
   * dpadr.onTrue(elevator.gotoL3());
   * lsb.onTrue(elevator.release());
   * rsb.onTrue(elevator.idle());
   */
  // lt.whileTrue(elevator.manualUp());
  // rt.whileTrue(elevator.manualDown());
  // dpadl.whileTrue(elevator.manualIn());
  // dpadr.whileTrue(elevator.manualOut());  
  // y.whileTrue(elevator.manualIntake());
  lsb.onTrue(elevator.release()); 
  rsb.onTrue(elevator.idle());
  lt.whileTrue(elevator.gotoL4());
  rt.onTrue(elevator.gotoL2());


  rb.whileTrue(processor.manualDown());
  lb.whileTrue(processor.manualUp());
  x.whileTrue(processor.manualIn());
  b.whileTrue(processor.manualOut());
    //y.onTrue(processor.autointake());
    ////a.onTrue(processor.autoshoot());
    //x.onTrue(processor.store());
    dpadup.whileTrue(climber.climbCommand());
    dpaddn.whileTrue(climber.releaseCommand());

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

  public Command getAutonomousCommand() {
    //return m_chooser.getSelected();
    return Commands.deadline(
      waitSeconds(1.5),
      Commands.run(() -> {  drivetrain.drive(new ChassisSpeeds(1, 0, 0)); })
    )
    .andThen(() -> { drivetrain.drive(new ChassisSpeeds(0, 0, 0)); });
  }
}
