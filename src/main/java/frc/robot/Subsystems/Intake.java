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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Utils;

import frc.robot.IO.SparkMaxIO;

enum IntakeState { 
    INTAKE_STATE_UNKNOWN,
    INTAKE_STATE_RETRACTED,
    INTAKE_STATE_AMP,
    INTAKE_STATE_EXTENDED,
    INTAKE_STATE_MOVING
}


public class Intake extends SubsystemBase {
    private final SparkMaxIO angleMotorIo;
    private final SparkMaxIO intakeMotorIo;
    private final PIDController angController = new PIDController(1.5, 0, 0);
    private final SlewRateLimiter angLimiter = new SlewRateLimiter(2);



    private boolean manualIntakeControl = false;
    private final double intakeRotationSpeed = 0.2;
    private double manualMovementSpeed = 0.0;
    private IntakeState currentIntakeState;
    private IntakeState desiredIntakeState;
    public Intake(SparkMaxIO angleMotorIo, SparkMaxIO intakeMotorIo) {
        this.angleMotorIo = angleMotorIo;
        this.intakeMotorIo = intakeMotorIo;
        currentIntakeState = IntakeState.INTAKE_STATE_UNKNOWN;
        desiredIntakeState = IntakeState.INTAKE_STATE_RETRACTED;
    }

    private void UpdateState(){
        double encoderAngle = angleMotorIo.getPosition();
        if (manualIntakeControl && manualMovementSpeed > 0){
            currentIntakeState = IntakeState.INTAKE_STATE_MOVING;
            return;
        }

        if (Utils.IsDoubleApproximately(encoderAngle, Constants.Intake.retractedSetPoint, Constants.Delta)){
            currentIntakeState = IntakeState.INTAKE_STATE_RETRACTED;
        } else if (Utils.IsDoubleApproximately(encoderAngle, Constants.Intake.ampSetPoint, 0.3)){
            currentIntakeState = IntakeState.INTAKE_STATE_AMP;
        } else if (Utils.IsDoubleApproximately(encoderAngle, Constants.Intake.extendedSetPoint, Constants.Delta)){
            currentIntakeState = IntakeState.INTAKE_STATE_EXTENDED;
        } else {
            currentIntakeState = IntakeState.INTAKE_STATE_MOVING;
        }
    }

    private double GetDesiredPosition(IntakeState intakeState){
        if (intakeState == IntakeState.INTAKE_STATE_RETRACTED){
            return Constants.Intake.retractedSetPoint;
        } else if (intakeState == IntakeState.INTAKE_STATE_AMP){
            return Constants.Intake.ampSetPoint;
        } else if (intakeState == IntakeState.INTAKE_STATE_EXTENDED){
            return Constants.Intake.extendedSetPoint;
        }
        return Constants.Intake.retractedSetPoint;
    }
    private void SetDesiredState(IntakeState newDesiredIntakeState){
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

    public IntakeState Update(
        boolean retractButtonPressed, boolean ampButtonPressed,
      boolean extendButtonPressed, boolean raiseButtonPressed, 
      boolean lowerButtonPressed, double rightTriggerAxis, double leftTriggerAxis,
      double ampShootSpeed, double speakerShootSpeed){
        if (retractButtonPressed){
            SetDesiredState(IntakeState.INTAKE_STATE_RETRACTED);
        } else if (ampButtonPressed){
            SetDesiredState(IntakeState.INTAKE_STATE_AMP);
        } else if (extendButtonPressed){
            SetDesiredState(IntakeState.INTAKE_STATE_EXTENDED);
        } else if (raiseButtonPressed){
            ManualUpdate(intakeRotationSpeed);
        } else if (lowerButtonPressed){
            ManualUpdate(-intakeRotationSpeed);
        } else if (manualIntakeControl){
            ManualUpdate(0);
        }
        RunLoop();
        if (rightTriggerAxis > 0.5){
            if (currentIntakeState == IntakeState.INTAKE_STATE_AMP){
                intakeMotorIo.setSpeed(ampShootSpeed);
            } else {
                intakeMotorIo.setSpeed(speakerShootSpeed);
            }
        } else {
            intakeMotorIo.setSpeed(leftTriggerAxis * 0.5);
        }
        return currentIntakeState;
    }

}
