package frc.robot.Subsystems.CoralHandler;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.wpilibj2.command.Commands.runEnd;

import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final SparkMax liftMotor;
    private final RelativeEncoder liftEncoder;
    private final PIDController fastLiftController = new PIDController(Constants.Elevator.Lift.kFastP, Constants.Elevator.Lift.kFastI, Constants.Elevator.Lift.kFastD);
    private final PIDController slowLiftController = new PIDController(Constants.Elevator.Lift.kSlowP, Constants.Elevator.Lift.kSlowI, Constants.Elevator.Lift.kSlowD);
    private final SlewRateLimiter liftLimiter = new SlewRateLimiter(2);

    public Elevator(int liftMotorId) {
        liftMotor = new SparkMax(liftMotorId, MotorType.kBrushless);
        liftEncoder = liftMotor.getAlternateEncoder();
    }

    public void liftToQuickly(double setpoint) {
        var speed = -MathUtil.clamp(liftLimiter.calculate(fastLiftController.calculate(liftEncoder.getPosition(), setpoint)), -1, 1);
        SmartDashboard.putNumber("elevator", speed);
        liftMotor.set(speed);
    }

    public boolean isAtFastSetpoint() {
        return fastLiftController.atSetpoint();
    }

    public void liftToSlowly(double setpoint) {
        var speed = -MathUtil.clamp(liftLimiter.calculate(slowLiftController.calculate(liftEncoder.getPosition(), setpoint)), -1, 1);
        SmartDashboard.putNumber("elevator", speed);
        liftMotor.set(speed);
    }

    public boolean isAtSlowSetpoint() {
        return slowLiftController.atSetpoint();
    }

    public Command liftToQuicklyCommand(double setpoint) {
        return run(() -> liftToQuickly(setpoint));
    }

    public Command liftToSlowlyCommand(double setpoint) {
        return run(() -> liftToSlowly(setpoint));
    }

    public Command manualUp() {
        return runEnd(() -> liftMotor.set(-0.2), () -> liftMotor.set(-0.075));
    }

    public Command manualDown() {
        return runEnd(() -> liftMotor.set(0), () -> liftMotor.set(-0.075));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elevator sensor", liftEncoder.getPosition());
    }
}
