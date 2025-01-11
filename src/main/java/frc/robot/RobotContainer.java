// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class RobotContainer {

  //private Robot robot;

  

  SendableChooser<Command> m_chooser = new SendableChooser<>();


  CommandXboxController DriveController = new CommandXboxController(0);
  //SmartDashboard
  //CommandXboxController coDriveController = new CommandXboxController(1);
  //Trigger xButton = DriveController.x();  
  //Trigger yButton = DriveController.y();  
  //Trigger bButton = DriveController.b();  
  //Trigger ltrigger = DriveController.leftTrigger();
  //Trigger rtrigger = DriveController.rightTrigger();
  //Trigger noteAquired = new Trigger(IntakeSubsystem.getInstance()::getHasNote);
  //Trigger dPadUp = DriveController.povUp();
  //Trigger dPadDown = DriveController.povDown();

  public RobotContainer() {

    //robot = new_robot;
    configureBindings();
    //m_chooser.setDefaultOption("taxi", robot.taxi);
    //m_chooser.addOption("shoot", robot.shoot);
    //m_chooser.addOption("shoot and taxi", robot.shoot.andThen(robot.taxi));
    //SmartDashboard.putData(m_chooser);
  }

  private void configureBindings() {

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
    return m_chooser.getSelected();
  }
}
