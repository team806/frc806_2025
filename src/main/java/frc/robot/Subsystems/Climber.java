package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    SparkFlex driveMotor;

    public Climber() {
        driveMotor = new SparkFlex(Constants.Climber.MotorID, MotorType.kBrushless);
        driveMotor.configure(Configs.climber.climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
