package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    SparkFlex driveMotor;

    public Climber(int driveMotorID) {
        driveMotor = new SparkFlex(driveMotorID, MotorType.kBrushless);
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.smartCurrentLimit(Constants.Climber.CurrentLimit);
        driveConfig.idleMode(IdleMode.kBrake);
        driveConfig.smartCurrentLimit(80);
        //driveConfig.softLimit.forwardSoftLimitEnabled(true);
        //driveConfig.softLimit.forwardSoftLimit(-8);
        //driveConfig.softLimit.reverseSoftLimitEnabled(true);
        //driveConfig.softLimit.reverseSoftLimit(-150);
        
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command climbCommand() {
        return this.startEnd(
            () -> { driveMotor.set(Constants.Climber.ClimbSpeed); },
            () -> { driveMotor.stopMotor(); }
        );
    }

    public Command releaseCommand() {
        return this.startEnd(
            () -> { driveMotor.set(Constants.Climber.ReleaseSpeed); },
            () -> { driveMotor.stopMotor(); }
        );
    }

    public void disable() {
        driveMotor.disable();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("");
        builder.setActuator(true);
        builder.setSafeState(this::disable);
        builder.addDoubleProperty("MotorSpeed", () -> driveMotor.get(), null);
        builder.addBooleanProperty("IsClimbing", () -> driveMotor.get() == Constants.Climber.ClimbSpeed, null);
        builder.addBooleanProperty("IsReleasing", () -> driveMotor.get() == Constants.Climber.ReleaseSpeed, null);
    }
}
