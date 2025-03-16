package frc.robot.Subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utils;


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
    private final SparkAbsoluteEncoder angleEncoder;
    private final PIDController angController = new PIDController(1.5, 0, 0);
    private final SlewRateLimiter angLimiter = new SlewRateLimiter(2);




    private boolean manualIntakeControl = false;
    private final double intakeRotationSpeed = 0.2;
    private double manualMovementSpeed = 0.0;
    private ProcessorState currentIntakeState;
    private ProcessorState desiredIntakeState;
    public Processor() {
        angleMotor = new SparkFlex(Constants.Pconstants.angID, MotorType.kBrushless);
        intakeMotor = new SparkFlex(Constants.Pconstants.intakeID, MotorType.kBrushless);
        algaeSensor = new DigitalInput(0);
            
        angleEncoder = angleMotor.getAbsoluteEncoder();
        currentIntakeState = ProcessorState.STATE_UNKNOWN;
        desiredIntakeState = ProcessorState.STATE_RETRACTED;
    }

    private void UpdateState(){
        double encoderAngle = angleEncoder.getPosition();
        if (manualIntakeControl && manualMovementSpeed > 0){
            currentIntakeState = ProcessorState.STATE_MOVING;
            return;
        }

        if (Utils.IsDoubleApproximately(encoderAngle, Constants.Pconstants.retractedSetPoint, Constants.Delta)){
            currentIntakeState = ProcessorState.STATE_RETRACTED;
        } else if (Utils.IsDoubleApproximately(encoderAngle, Constants.Pconstants.ampSetPoint, 0.3)){
            currentIntakeState = ProcessorState.STATE_AMP;
        } else if (Utils.IsDoubleApproximately(encoderAngle, Constants.Pconstants.extendedSetPoint, Constants.Delta)){
            currentIntakeState = ProcessorState.STATE_EXTENDED;
        } else {
            currentIntakeState = ProcessorState.STATE_MOVING;
        }
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
        manualIntakeControl = false;
        desiredIntakeState = newDesiredIntakeState;
        angController.setSetpoint(GetDesiredPosition(desiredIntakeState));
    }

    private void ManualUpdate(double speed){
        manualIntakeControl = true;
        manualMovementSpeed = speed;
    }

    private void RunLoop(){
        if (manualIntakeControl){
            angleMotor.set(manualMovementSpeed);
        } else {
            if (currentIntakeState == desiredIntakeState){
                angleMotor.set(0);
            } else {
                angleMotor.set(angLimiter.calculate(angController.calculate(angleEncoder.getPosition())));
            }
        }
        UpdateState();
    }
    public ProcessorState Update(
        boolean raiseButtonPressed, 
      boolean lowerButtonPressed, double rightTriggerAxis, double leftTriggerAxis,
      double inSpeed, double shootSpeed){
        if (raiseButtonPressed){
            ManualUpdate(intakeRotationSpeed);
        } else if (lowerButtonPressed){
            ManualUpdate(-intakeRotationSpeed);
        } else if (manualIntakeControl){
            ManualUpdate(0);
        }
        RunLoop();
        if (rightTriggerAxis > 0.5){
            intakeMotor.set(inSpeed);
        } else {
            intakeMotor.set(leftTriggerAxis * 0.5);
        }
        return currentIntakeState;
    }

    public Command extend(){
        return this.run(()-> SetDesiredState(ProcessorState.STATE_EXTENDED));
    }

    public Command transport(){
        return this.run(()-> SetDesiredState(ProcessorState.STATE_TRANSPORT));
    }

    public Command store(){
        return this.run(()-> SetDesiredState(ProcessorState.STATE_RETRACTED));
    }

    public Command score(){
        return this.run(()-> SetDesiredState(ProcessorState.STATE_AMP));
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
        return this.run(()-> { intakeMotor.set(0.1); }).until(() -> !algaeSensor.get());
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

    @Override
    public void periodic() {
        
        //sensor = algaeSensor.get();
    }

    
}