package frc.robot.Subsystems.CoralHandler;

import java.util.Map;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.deadline;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.race;
import static edu.wpi.first.wpilibj2.command.Commands.select;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public final class CoralHandler extends SubsystemBase {
    private final Elevator elevator;
    private final Arm arm;
    private final SparkMax intakeMotor;
    private final DigitalInput coralSensor;

    private enum ElevatorPosition {
        IDLE,
        INTAKE,
        L1,
        A1,
        L2,
        A2,
        L3,
        L4
    }

    private ElevatorPosition position = ElevatorPosition.IDLE;

    private double elevatorIdlePosition = Constants.Elevator.Lift.IdlePosition;
    private double armIdlePosition = Constants.Elevator.Arm.IdlePosition;

    public CoralHandler(int liftMotorId, int armMotorId, int intakeMotorId, int coralSensorId) {
        elevator = new Elevator(liftMotorId);
        arm = new Arm(armMotorId);
        intakeMotor = new SparkMax(intakeMotorId, MotorType.kBrushless);
        coralSensor = new DigitalInput(coralSensorId);
        setDefaultCommand(idle());
    }

    public Command idle() {
        return parallel(
            either(
                race(
                    waitSeconds(Constants.Elevator.Intake.IntakeTimeout),
                    waitUntil(() -> !coralSensor.get())
                ).andThen(runOnce(() -> intakeMotor.set(0))),
                runOnce(() -> intakeMotor.set(0)),
                () -> intakeMotor.get() == Constants.Elevator.Intake.IntakeSpeed
            ),
            arm.driveAngleToCommand(Constants.Elevator.Arm.IdlePosition),
            elevator.idleDrop()
        ).withName("Idle");
    }

    public Command intakeAndHold() {
        return parallel(
            elevator.liftToQuicklyCommand(Constants.Elevator.Lift.IntakePosition),
            arm.driveAngleToCommand(Constants.Elevator.Arm.IntakePosition),
            runOnce(() -> intakeMotor.set(Constants.Elevator.Intake.IntakeSpeed))
        ).withName("Intake");
    }

    public Command gotoL1() {
        return parallel(
            elevator.liftToQuicklyCommand(Constants.Elevator.Lift.L1PrepPosition),
            arm.driveAngleToCommand(Constants.Elevator.Arm.L1PrepPosition),
            run(() -> {
                if (elevator.isAtFastSetpoint() && arm.isAtSetpoint()) {
                    position = ElevatorPosition.L1;
                }
            })
        ).withName("Going to L1");
    }

    public Command gotoA1() {
        return parallel(
            elevator.liftToQuicklyCommand(Constants.Elevator.Lift.A1PrepPosition),
            arm.driveAngleToCommand(Constants.Elevator.Arm.A1PrepPosition)
            //run(() -> System.out.println("Test"))
        )//.until(() -> /*arm.isAtSetpoint() && */elevator.isAtFastSetpoint())
        /*.andThen(runOnce(() -> position = ElevatorPosition.A1))*/.withName("Going to A1");
    }

    public Command gotoL2() {
        return parallel(
            elevator.liftToQuicklyCommand(Constants.Elevator.Lift.L2PrepPosition),
            arm.driveAngleToCommand(Constants.Elevator.Arm.L2PrepPosition),
            run(() -> {
                if (elevator.isAtFastSetpoint() && arm.isAtSetpoint()) {
                    position = ElevatorPosition.L2;
                }
            })
        ).withName("Going to L2");
    }

    public Command gotoA2() {
        return parallel(
            elevator.liftToQuicklyCommand(Constants.Elevator.Lift.A2PrepPosition),
            arm.driveAngleToCommand(Constants.Elevator.Arm.A2PrepPosition)
        ).until(() -> arm.isAtSetpoint() && elevator.isAtFastSetpoint())
        .andThen(runOnce(() -> position = ElevatorPosition.A2)).withName("Going to A2");
    }

    public Command gotoL3() {
        return parallel(
            elevator.liftToQuicklyCommand(Constants.Elevator.Lift.L3PrepPosition),
            arm.driveAngleToCommand(Constants.Elevator.Arm.L3PrepPosition),
            run(() -> {
                if (elevator.isAtFastSetpoint() && arm.isAtSetpoint()) {
                    position = ElevatorPosition.L3;
                }
            })
        ).withName("Going to L3");
    }

    public Command gotoL4() {
        return parallel(
            elevator.liftToQuicklyCommand(Constants.Elevator.Lift.L4PrepPosition),
            arm.driveAngleToCommand(Constants.Elevator.Arm.L4PrepPosition),
            run(() -> {
                if (elevator.isAtFastSetpoint() && arm.isAtSetpoint()) {
                    position = ElevatorPosition.L4;
                }
            })
        ).withName("Going to L4");
    }

    public Command release() {
        return select(
            Map.of(
                ElevatorPosition.L1, releaseL1(),
                ElevatorPosition.A1, releaseA1(),
                ElevatorPosition.L2, releaseL2(),
                ElevatorPosition.A2, releaseA2(),
                ElevatorPosition.L3, releaseL3(),
                ElevatorPosition.L4, releaseL4()
            ),
            () -> position);
    }

    public Command releaseL1() {
        return deadline(
            waitSeconds(Constants.Elevator.Intake.ReleaseTime),
            elevator.liftToQuicklyCommand(Constants.Elevator.Lift.L1PrepPosition),
            arm.driveAngleToCommand(Constants.Elevator.Arm.L1PrepPosition),
            runOnce(() -> {
                intakeMotor.set(Constants.Elevator.Intake.ReleaseSpeed);
                elevatorIdlePosition = Constants.Elevator.Lift.L1PrepPosition;
                armIdlePosition = Constants.Elevator.Arm.L1PrepPosition;
            })
        ).withName("Releasing").andThen(parallel(
            runOnce(() -> intakeMotor.set(0)),
            elevator.liftToQuicklyCommand(Constants.Elevator.Lift.L1PrepPosition),
            arm.driveAngleToCommand(Constants.Elevator.Arm.L1PrepPosition)
        )).withName("Holding at L1");
    }

    public Command releaseA1() {
        return parallel(
            runOnce(() -> intakeMotor.set(Constants.Elevator.Intake.AlgaeSpeed)),
            arm.driveAngleToCommand(Constants.Elevator.Arm.A1ReleasePosition)
        ).until(() -> arm.isAtSetpoint()).withName("Releasing A1");
    }

    public Command releaseL2() {
        return deadline(
            waitSeconds(Constants.Elevator.Intake.ReleaseTime),
            elevator.liftToQuicklyCommand(Constants.Elevator.Lift.L2PrepPosition),
            arm.driveAngleToCommand(Constants.Elevator.Arm.L2PrepPosition),
            runOnce(() -> {
                intakeMotor.set(Constants.Elevator.Intake.ReleaseSpeed);
                elevatorIdlePosition = Constants.Elevator.Lift.L2PrepPosition;
                armIdlePosition = Constants.Elevator.Arm.L2PrepPosition;
            })
        ).withName("Releasing").andThen(parallel(
            runOnce(() -> intakeMotor.set(0)),
            elevator.liftToQuicklyCommand(Constants.Elevator.Lift.L2PrepPosition),
            arm.driveAngleToCommand(Constants.Elevator.Arm.L2PrepPosition)
        )).withName("Holding at L2");
    }

    public Command releaseA2() {
        return parallel(
            runOnce(() -> intakeMotor.set(Constants.Elevator.Intake.AlgaeSpeed)),
            arm.driveAngleToCommand(Constants.Elevator.Arm.A2ReleasePosition)
        ).until(() -> arm.isAtSetpoint()).withName("Releasing A2");
    }

    public Command releaseL3() {
        return deadline(
            waitSeconds(Constants.Elevator.Intake.ReleaseTime),
            elevator.liftToQuicklyCommand(Constants.Elevator.Lift.L3PrepPosition),
            arm.driveAngleToCommand(Constants.Elevator.Arm.L3PrepPosition),
            runOnce(() -> {
                intakeMotor.set(Constants.Elevator.Intake.ReleaseSpeed);
                elevatorIdlePosition = Constants.Elevator.Lift.L3PrepPosition;
                armIdlePosition = Constants.Elevator.Arm.L3PrepPosition;
            })
        ).withName("Releasing").andThen(parallel(
            runOnce(() -> intakeMotor.set(0)),
            elevator.liftToQuicklyCommand(Constants.Elevator.Lift.L3PrepPosition),
            arm.driveAngleToCommand(Constants.Elevator.Arm.L3PrepPosition)
        )).withName("Holding at L3");
    }

    public Command releaseL4() {
        return deadline(
            waitSeconds(Constants.Elevator.Intake.ReleaseTime),
            elevator.liftToQuicklyCommand(Constants.Elevator.Lift.L4PrepPosition),
            arm.driveAngleToCommand(Constants.Elevator.Arm.L4PrepPosition),
            runOnce(() -> {
                intakeMotor.set(Constants.Elevator.Intake.ReleaseSpeed);
                elevatorIdlePosition = Constants.Elevator.Lift.L4PrepPosition;
                armIdlePosition = Constants.Elevator.Arm.L4PrepPosition;
            })
        ).withName("Releasing").andThen(parallel(
            runOnce(() -> intakeMotor.set(0)),
            elevator.liftToQuicklyCommand(Constants.Elevator.Lift.L4PrepPosition),
            arm.driveAngleToCommand(Constants.Elevator.Arm.L4PrepPosition)
        )).withName("Holding at L4");
    }

    public Command manualUp() {
        return elevator.manualUp();
    }

    public Command manualDown() {
        return elevator.manualDown();
    }
    public Command manualOut() {
        return arm.manualOut();
    }

    public Command manualIn() {
        return arm.manualIn();
    }

    public Command manualIntake() {
        return this.runEnd(() -> intakeMotor.set(0.5),
                        ()->intakeMotor.set(0));
    }

    public Command manualShoot(){
        return this.runEnd(()->intakeMotor.set(-0.5),
                        ()->intakeMotor.set(0));
    }

    @Override
    public void periodic() {
       SmartDashboard.putString("CH command", getCurrentCommand() != null ? getCurrentCommand().getName() : "");
       SmartDashboard.putString("elevator position", position.toString());
    }
}
