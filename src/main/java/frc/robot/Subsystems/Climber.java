package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    SparkMax driveMotor;

    public Climber(int driveMotorID) {
        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.smartCurrentLimit(Constants.Climber.currentLimit);
        driveConfig.idleMode(IdleMode.kBrake);
        driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command climbCommand() {
        return this.startEnd(
            () -> { driveMotor.set(Constants.Climber.climbSpeed); },
            () -> { driveMotor.stopMotor(); }
        );
    }

    public Command releaseCommand() {
        return this.startEnd(
            () -> { driveMotor.set(Constants.Climber.releaseSpeed); },
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
        builder.addBooleanProperty("IsClimbing", () -> driveMotor.get() == Constants.Climber.climbSpeed, null);
        builder.addBooleanProperty("IsReleasing", () -> driveMotor.get() == Constants.Climber.releaseSpeed, null);
    }
}
