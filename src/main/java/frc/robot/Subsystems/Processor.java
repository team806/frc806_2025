package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Utils;
import frc.robot.Constants.Intake;
import frc.robot.IO.SparkMaxIO;

enum ProcessorState { 
    STATE_UNKNOWN,
    STATE_RETRACTED,
    STATE_AMP,
    STATE_EXTENDED,
    STATE_MOVING
}


public class Processor extends SubsystemBase {
    private final SparkMaxIO angleMotorIo;
    private final SparkMaxIO intakeMotorIo;
    private final PIDController angController = new PIDController(1.5, 0, 0);
    private final SlewRateLimiter angLimiter = new SlewRateLimiter(2);



    private boolean manualIntakeControl = false;
    private final double intakeRotationSpeed = 0.2;
    private double manualMovementSpeed = 0.0;
    private ProcessorState currentIntakeState;
    private ProcessorState desiredIntakeState;
    public Processor(SparkMaxIO angleMotorIo, SparkMaxIO intakeMotorIo) {
        this.angleMotorIo = angleMotorIo;
        this.intakeMotorIo = intakeMotorIo;
        currentIntakeState = ProcessorState.STATE_UNKNOWN;
        desiredIntakeState = ProcessorState.STATE_RETRACTED;
    }

    private void UpdateState(){
        double encoderAngle = angleMotorIo.getPosition();
        if (manualIntakeControl && manualMovementSpeed > 0){
            currentIntakeState = ProcessorState.STATE_MOVING;
            return;
        }

        if (Utils.IsDoubleApproximately(encoderAngle, Constants.Intake.retractedSetPoint, Constants.Delta)){
            currentIntakeState = ProcessorState.STATE_RETRACTED;
        } else if (Utils.IsDoubleApproximately(encoderAngle, Constants.Intake.ampSetPoint, 0.3)){
            currentIntakeState = ProcessorState.STATE_AMP;
        } else if (Utils.IsDoubleApproximately(encoderAngle, Constants.Intake.extendedSetPoint, Constants.Delta)){
            currentIntakeState = ProcessorState.STATE_EXTENDED;
        } else {
            currentIntakeState = ProcessorState.STATE_MOVING;
        }
    }

    private double GetDesiredPosition(ProcessorState intakeState){
        if (intakeState == ProcessorState.STATE_RETRACTED){
            return Constants.Intake.retractedSetPoint;
        } else if (intakeState == ProcessorState.STATE_AMP){
            return Constants.Intake.ampSetPoint;
        } else if (intakeState == ProcessorState.STATE_EXTENDED){
            return Constants.Intake.extendedSetPoint;
        }
        return Constants.Intake.retractedSetPoint;
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
            angleMotorIo.setSpeed(manualMovementSpeed);
        } else {
            if (currentIntakeState == desiredIntakeState){
                angleMotorIo.setSpeed(0);
            } else {
                angleMotorIo.setSpeed(angLimiter.calculate(angController.calculate(angleMotorIo.getPosition())));
            }
        }
        UpdateState();
    }
    public ProcessorState Update(
        boolean retractButtonPressed, boolean ampButtonPressed,
      boolean extendButtonPressed, boolean raiseButtonPressed, 
      boolean lowerButtonPressed, double rightTriggerAxis, double leftTriggerAxis,
      double inSpeed, double shootSpeed){
        if (retractButtonPressed){
            SetDesiredState(ProcessorState.STATE_RETRACTED);
        } else if (ampButtonPressed){
            SetDesiredState(ProcessorState.STATE_AMP);
        } else if (extendButtonPressed){
            SetDesiredState(ProcessorState.STATE_EXTENDED);
        } else if (raiseButtonPressed){
            ManualUpdate(intakeRotationSpeed);
        } else if (lowerButtonPressed){
            ManualUpdate(-intakeRotationSpeed);
        } else if (manualIntakeControl){
            ManualUpdate(0);
        }
        RunLoop();
        if (rightTriggerAxis > 0.5){
            intakeMotorIo.setSpeed(inSpeed);
        } else {
            intakeMotorIo.setSpeed(leftTriggerAxis * 0.5);
        }
        return currentIntakeState;
    }

    public Command extendAndIntake =
    Commands.runOnce(()->{SetDesiredState(ProcessorState.STATE_EXTENDED);})
    .andThen(Commands.waitSeconds(0.25))
    .andThen(Commands.runOnce(()->{intakeMotorIo.setSpeed(1.0);;})); //FIXME Idk why this keeps giving me an error. Im also not great with commands -Owen

}
