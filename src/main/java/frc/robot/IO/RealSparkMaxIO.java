package frc.robot.IO;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class RealSparkMaxIO implements SparkMaxIO {
    public static enum EncoderType{
        ENCODER_TYPE_DEFAULT,
        ENCODER_TYPE_ALTERNATE,
        ENCODER_TYPE_ABSOLUTE
    }
    private EncoderType encoderType;
    private SparkMax motor;

    public RealSparkMaxIO(int pin, MotorType motorType) {
        motor = new SparkMax(pin, motorType);
        encoderType = EncoderType.ENCODER_TYPE_DEFAULT;
    }

    public RealSparkMaxIO(int pin, MotorType motorType, EncoderType encoderType){
        motor = new SparkMax(pin, motorType);
        this.encoderType = encoderType;
    }

    @Override
    public void configure(SparkMaxConfig config, ResetMode resetMode, PersistMode persistMode) {
        motor.configure(config, resetMode, persistMode);
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public double getPosition() {
        switch (encoderType) {
            case ENCODER_TYPE_DEFAULT:
                return motor.getEncoder().getPosition();
            case ENCODER_TYPE_ABSOLUTE:
                return motor.getAbsoluteEncoder().getPosition();
            case ENCODER_TYPE_ALTERNATE:
                return motor.getAlternateEncoder().getPosition();
            default:
                return motor.getEncoder().getPosition();
        }
    }
}
