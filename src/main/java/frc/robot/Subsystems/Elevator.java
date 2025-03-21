package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.race;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final PIDController fastLiftController = new PIDController(Constants.Elevator.Lift.kFastP, Constants.Elevator.Lift.kFastI, Constants.Elevator.Lift.kFastD);
    private final PIDController slowLiftController = new PIDController(Constants.Elevator.Lift.kSlowP, Constants.Elevator.Lift.kSlowI, Constants.Elevator.Lift.kSlowD);
    private final PIDController armController = new PIDController(Constants.Elevator.Arm.kP, Constants.Elevator.Arm.kI, Constants.Elevator.Arm.kD);

    private final SparkMax liftMotor;
    //private final SparkMax followMotor;
    private final RelativeEncoder liftEncoder;
    private final SparkFlex armMotor;
    private final SparkAbsoluteEncoder armEncoder;
    private final SparkMax intakeMotor;
    private final DigitalInput coralSensor;
    
    private final SlewRateLimiter liftLimiter = new SlewRateLimiter(2);
    private final SlewRateLimiter angLimiter = new SlewRateLimiter(2);

    //private double armdownspeed;
    //private double armupspeed;
    //private double armholdspeed;

    public Elevator(int liftMotorId, int armMotorId, int intakeMotorId, int coralSensorPort) {
        liftMotor = new SparkMax(liftMotorId, MotorType .kBrushless);
        liftEncoder = liftMotor.getEncoder();
        //followMotor = new SparkMax(15, MotorType.kBrushless);
    

        armMotor = new SparkFlex(armMotorId, MotorType.kBrushless);
        //armEncoder = new CANcoder(armEncoderId, "Default Name");
        armEncoder = armMotor.getAbsoluteEncoder();

        intakeMotor = new SparkMax(intakeMotorId, MotorType.kBrushless);

        coralSensor = new DigitalInput(coralSensorPort);
    }

    private void liftToQuickly(double setpoint) {
        armMotor.set(MathUtil.clamp(angLimiter.calculate(fastLiftController.calculate(armEncoder.getPosition(), setpoint)), -1, 1));
    }

    private void liftToSlowly(double setpoint) {
        armMotor.set(MathUtil.clamp(angLimiter.calculate(slowLiftController.calculate(armEncoder.getPosition(), setpoint)), -1, 1));
    }

    private void driveAngleTo(double setpoint) {
        armMotor.set(MathUtil.clamp(angLimiter.calculate(armController.calculate(armEncoder.getPosition(), setpoint)), -1, 1));
    }

    public Command idle() {
        return parallel(
            runOnce(() -> { intakeMotor.set(0);}),
            run(() -> { driveAngleTo(Constants.Elevator.Arm.IdlePosition); }),
            run(() -> { liftToSlowly(Constants.Elevator.Lift.IdlePosition); })
        ).withName("Idle");
    }

    public Command intakeAndHold() {
        return parallel(
            runOnce(() -> { intakeMotor.set(Constants.Elevator.Intake.IntakeSpeed); }),
            run(() -> { liftToQuickly(Constants.Elevator.Lift.IntakePosition); }),
            run(() -> { driveAngleTo(Constants.Elevator.Arm.IntakePosition); })
        ).until(() -> armController.atSetpoint() && fastLiftController.atSetpoint()).withName("Preparing to intake")
        .andThen(
            race(
                waitSeconds(Constants.Elevator.Intake.IntakeTimeout),
                waitUntil(() -> !coralSensor.get())
            )
        ).withName("Intake")
        .andThen(
            either(
                none().withName("Failed to intake"),
                runOnce(() -> { intakeMotor.set(Constants.Elevator.Intake.HoldSpeed); }).withName("Holding coral"),
                coralSensor::get
            )
        );
        
        // Drive lift to coral station height, drive arm to coral station angle
        // Rotate instake
        // Wait for timeout or coral sensor
        // Timeout, stop intake and leave
        // Hit, stop intake
    }

    public Command gotoL1() {
        return parallel(
            run(() -> { liftToQuickly(Constants.Elevator.Lift.L1PrepPosition); }),
            run(() -> { driveAngleTo(Constants.Elevator.Arm.L1PrepPosition); })
        ).until(() -> armController.atSetpoint() && fastLiftController.atSetpoint()).withName("Going to L1");
    }

    public Command gotoA1() {
        return parallel(
            run(() -> { liftToQuickly(Constants.Elevator.Lift.A1PrepPosition); }),
            run(() -> { driveAngleTo(Constants.Elevator.Arm.A1PrepPosition); })
        ).until(() -> armController.atSetpoint() && fastLiftController.atSetpoint()).withName("Going to A1");
    }

    public Command gotoL2() {
        return parallel(
            run(() -> { liftToQuickly(Constants.Elevator.Lift.L2PrepPosition); }),
            run(() -> { driveAngleTo(Constants.Elevator.Arm.L2PrepPosition); })
        ).until(() -> armController.atSetpoint() && fastLiftController.atSetpoint()).withName("Going to L2");
    }

    public Command gotoA2() {
        return parallel(
            run(() -> { liftToQuickly(Constants.Elevator.Lift.A2PrepPosition); }),
            run(() -> { driveAngleTo(Constants.Elevator.Arm.A2PrepPosition); })
        ).until(() -> armController.atSetpoint() && fastLiftController.atSetpoint()).withName("Going to A2");
    }

    public Command gotoL3() {
        return parallel(
            run(() -> { liftToQuickly(Constants.Elevator.Lift.L3PrepPosition); }),
            run(() -> { driveAngleTo(Constants.Elevator.Arm.L3PrepPosition); })
        ).until(() -> armController.atSetpoint() && fastLiftController.atSetpoint()).withName("Going to L3");
    }

    public Command gotoL4() {
        return parallel(
            run(() -> { liftToQuickly(Constants.Elevator.Lift.L4PrepPosition); }),
            run(() -> { driveAngleTo(Constants.Elevator.Arm.L4PrepPosition); })
        ).until(() -> armController.atSetpoint() && fastLiftController.atSetpoint()).withName("Going to L4");
    }

    public Command release() {
        return run(()->intakeMotor.set(-Constants.Elevator.Intake.IntakeSpeed))
        .andThen(
            race(
                waitSeconds(Constants.Elevator.Intake.IntakeTimeout),
                waitUntil(() -> coralSensor.get())));
                
    }
    public Command manualUp(){
        return this.runEnd(()->{liftMotor.set(-0.2);},
                        ()->{liftMotor.set(-0.075);});
    }

    public Command manualDown(){
        return this.runEnd(()->{liftMotor.set(0);},
                        ()->{liftMotor.set(-0.075);});
    }
    public Command manualOut(){
        return this.runEnd(()->{armMotor.set(0.2);},
                        ()->{armMotor.set(-0.04);});
    }

    public Command manualIn(){
        return this.runEnd(()->{armMotor.set(-0.2);},
                        ()->{armMotor.set(-0.04);});
    }

    public Command manualIntake(){
        return this.runEnd(()->{intakeMotor.set(0.5);},
                        ()->{intakeMotor.set(0);});
    }

    public Command manualShoot(){
        return this.runEnd(()->{intakeMotor.set(-0.5);},
                        ()->{intakeMotor.set(0);});
    }

    //@Override
    //public void periodic() {
    //    smartdash
    //
    //}
}
