package frc.robot.Subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.race;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Processor extends SubsystemBase {
    private final SparkFlex angleMotor;
    private final SparkFlex intakeMotor;
    private  DigitalInput algaeSensor;
    private final SparkAbsoluteEncoder angleEncoder;
    private final PIDController angController = new PIDController(3, 0, 0);
    private final SlewRateLimiter angLimiter = new SlewRateLimiter(2);

    public Processor() {
        angleMotor = new SparkFlex(Constants.Pconstants.angID, MotorType.kBrushless);
        angleEncoder = angleMotor.getAbsoluteEncoder();
        intakeMotor = new SparkFlex(Constants.Pconstants.intakeID, MotorType.kBrushless);
        algaeSensor = new DigitalInput(0);

        angController.enableContinuousInput(0, 1);

        setDefaultCommand(idle());
    }

    public void driveAngleTo(double setpoint) {
        angleMotor.set(MathUtil.clamp(angLimiter.calculate(angController.calculate(angleEncoder.getPosition(), setpoint)), -1, 1));
    }

    public Command idle() {
        return parallel(
            runOnce(() -> { intakeMotor.set(0); }),
            run(() -> { driveAngleTo(Constants.AlgaeProcessor.IdlePosition); })
        )
        .withName("idle");
    }

    public Command intakeAndHold() {
        return parallel(
            runOnce(() -> { intakeMotor.set(1); }),
            run(() -> { driveAngleTo(Constants.AlgaeProcessor.IntakePosition); })
        ).until(() -> angController.atSetpoint()).withName("Preparing to intake")
        .andThen(
            race(
                waitSeconds(Constants.AlgaeProcessor.IntakeTimeout),
                waitUntil(() -> !algaeSensor.get())
            )
        ).withName("Intake")
        .andThen(
            either(
                runOnce(() -> { intakeMotor.set(0); }).withName("Failed to intake"),
                waitSeconds(Constants.AlgaeProcessor.InakeExtraHoldTime)
                    .andThen(parallel(
                        runOnce(() -> { intakeMotor.set(Constants.AlgaeProcessor.HoldSpeed); }),
                        run(() -> { driveAngleTo(Constants.AlgaeProcessor.HoldPosiiton); })
                    )).withName("Holding algae"),
                algaeSensor::get
            )
        );
    }

    public Command prepareToShoot() {
        return run(() -> { driveAngleTo(Constants.AlgaeProcessor.ShootPosition); }).withName("Preparing to shoot");
    }

    public Command shoot() {
        return runOnce(() -> { intakeMotor.set(Constants.AlgaeProcessor.ShootSpeed); })
            .andThen(waitSeconds(Constants.AlgaeProcessor.ShootTime))
            .andThen(() -> { intakeMotor.set(0); }).withName("Shooting");
    }    
}