package frc.robot.Subsystems.CoralHandler;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private final SparkFlex armMotor;
    private final SparkAbsoluteEncoder armEncoder;
    private final PIDController armController = new PIDController(Constants.Elevator.Arm.kP, Constants.Elevator.Arm.kI, Constants.Elevator.Arm.kD);
    private final SlewRateLimiter angLimiter = new SlewRateLimiter(2);

    public Arm(int armMotorId) {
        armMotor = new SparkFlex(armMotorId, MotorType.kBrushless);
        armEncoder = armMotor.getAbsoluteEncoder();

        armController.setTolerance(0.025);
    }

    public void driveAngleTo(double setpoint) {
        armMotor.set(-MathUtil.clamp(angLimiter.calculate(armController.calculate(armEncoder.getPosition(), setpoint)), -1, 1));
    }

    public boolean isAtSetpoint() {
        return armController.atSetpoint();
    }

    public Command driveAngleToCommand(double setpoint) {
        return run(() -> driveAngleTo(setpoint)).finallyDo(() -> armMotor.set(0));
    }

    public Command manualOut() {
        return runEnd(() -> armMotor.set(0.2), () -> armMotor.set(-0.04)).withName("manaulOut");
    }

    public Command manualIn() {
        return runEnd(() -> armMotor.set(-0.2), () -> armMotor.set(-0.04)).withName("manualIn");
    }

    public void stop() {
        armMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm sensor", armEncoder.getPosition());
        SmartDashboard.putString("arm command",
            getCurrentCommand() != null ?
                getCurrentCommand().getName() : "");
    }
}