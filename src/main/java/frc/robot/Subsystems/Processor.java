package frc.robot.Subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


enum ProcessorState { 
    STATE_UNKNOWN,
    STATE_RETRACTED,
    STATE_AMP,
    STATE_EXTENDED,
    STATE_MOVING, STATE_TRANSPORT
}


public class Processor extends SubsystemBase {
    private final SparkFlex angleMotor;
    private final SparkFlex intakeMotor;
    private  DigitalInput algaeSensor;
    private final SparkFlexConfig angleMotorConfig;
    private final SparkAbsoluteEncoder angleEncoder;
    private final PIDController angController = new PIDController(3, 0, 0);
    private final SlewRateLimiter angLimiter = new SlewRateLimiter(2);

    private ProcessorState currentIntakeState;
    private ProcessorState desiredIntakeState;
    public Processor() {
        angleMotor = new SparkFlex(Constants.Pconstants.angID, MotorType.kBrushless);
        angleMotorConfig = new SparkFlexConfig();
            angleMotorConfig.softLimit.forwardSoftLimitEnabled(false);
            angleMotorConfig.softLimit.forwardSoftLimit(-1.5);
            angleMotorConfig.softLimit.reverseSoftLimitEnabled(false);
            angleMotorConfig.softLimit.reverseSoftLimit(18);
    
        intakeMotor = new SparkFlex(Constants.Pconstants.intakeID, MotorType.kBrushless);
        algaeSensor = new DigitalInput(0);
            
        angleEncoder = angleMotor.getAbsoluteEncoder();
        currentIntakeState = ProcessorState.STATE_UNKNOWN;
        desiredIntakeState = ProcessorState.STATE_RETRACTED;

        angController.enableContinuousInput(-0.2, 0.2);
    }

    private double GetDesiredPosition(ProcessorState intakeState){
        if (intakeState == ProcessorState.STATE_RETRACTED){
            return Constants.Pconstants.retractedSetPoint;
        } else if (intakeState == ProcessorState.STATE_AMP){
            return Constants.Pconstants.ampSetPoint;
        } else if (intakeState == ProcessorState.STATE_EXTENDED){
            return Constants.Pconstants.extendedSetPoint;
        }
        return Constants.Pconstants.retractedSetPoint;
    }

    private void SetDesiredState(ProcessorState newDesiredIntakeState){
        desiredIntakeState = newDesiredIntakeState;
        angController.setSetpoint(GetDesiredPosition(desiredIntakeState));
    }

    public Command extend(){
        return this.run(()-> SetDesiredState(ProcessorState.STATE_EXTENDED))
                    .until(()-> angController.atSetpoint())
                    .andThen(()-> currentIntakeState = ProcessorState.STATE_EXTENDED);
    }

    public Command transport(){
        return this.run(()-> SetDesiredState(ProcessorState.STATE_TRANSPORT))
                    .until(()->angController.atSetpoint())
                    .andThen(()-> currentIntakeState = ProcessorState.STATE_TRANSPORT);
    }

    public Command store(){
        return this.run(()-> SetDesiredState(ProcessorState.STATE_RETRACTED))
                    .until(()->angController.atSetpoint())
                    .andThen(()-> currentIntakeState = ProcessorState.STATE_TRANSPORT);
    }

    public Command score(){
        return this.run(()-> SetDesiredState(ProcessorState.STATE_AMP))
                    .until(()->angController.atSetpoint())
                    .andThen(()-> currentIntakeState = ProcessorState.STATE_TRANSPORT);
    }

    public Command intake(){
        return run(()-> intakeMotor.set(0.5));
    }

    public Command shoot(){
        return this.run(()-> intakeMotor.set(-1));
    }

    public Command hold(){
        return this.run(()-> intakeMotor.set(0.1));
    }

    public Command testintake(){
        return run(()-> { intakeMotor.set(1); })
            .until(() -> !algaeSensor.get())
            .finallyDo(() -> { intakeMotor.set(0); });
    }

    public Command up() {
        return run(() -> {
            angController.setSetpoint(1);
            SmartDashboard.putNumber("setpoint", angController.getSetpoint());
            var speed = MathUtil.clamp(angLimiter.calculate(angController.calculate(angleEncoder.getPosition())), -1, 1);
            SmartDashboard.putNumber("speed", speed);
            angleMotor.set(speed);
        }).finallyDo(() -> { angleMotor.set(0); }).withName("up");
    }

    public Command down() {
        return run(() -> {
            angController.setSetpoint(0.7);
            SmartDashboard.putNumber("setpoint", angController.getSetpoint());
            var speed = MathUtil.clamp(angLimiter.calculate(angController.calculate(angleEncoder.getPosition())), -1, 1);
            SmartDashboard.putNumber("speed", speed);
            angleMotor.set(speed);
        }).finallyDo(() -> { angleMotor.set(0); }).withName("down");
    }


    public Command autointake(){
        return this.startRun(
            ()->SetDesiredState(ProcessorState.STATE_EXTENDED),
                 ()-> intake().until(algaeSensor::get)
                 .andThen(
                    Commands.parallel(
                        transport(),
                            hold()
                    )));
    }

    public Command autoshoot(){
        return this.startRun(
            ()->SetDesiredState(ProcessorState.STATE_AMP),
                ()->shoot()
                    .andThen(
                        store()
                    ));
             
    }

    public Command manualDown(){
        return runEnd(()->angleMotor.set(-0.2),()->{angleMotor.set(0.02);});
    }

    public Command manualUp(){
        return runEnd(()->angleMotor.set(0.2),()->{angleMotor.set(0.02);});
    }

    public Command manualIn(){
        return runEnd(()->intakeMotor.set(1),()->intakeMotor.set(0.1));
    }

    public Command manualOut(){
        return runEnd(()->intakeMotor.set(-0.75),()->intakeMotor.set(0.1));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("algae", algaeSensor.get());
        SmartDashboard.putNumber("algae encoder", angleEncoder.getPosition());
        SmartDashboard.putString("command", this.getCurrentCommand() != null ? this.getCurrentCommand().getName() : "");
    }

    
}