package frc.robot.Mocks;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.IO.SparkMaxIO;

public class MockSparkMaxIO implements SparkMaxIO {
    private double position;
    private double speed;

    @Override
    public void configure(SparkMaxConfig config, ResetMode resetMode, PersistMode persistMode) {
        return;
    }

    @Override
    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public double getSpeed(){
        return speed;
    }

    public void setPosition(double position){
        this.position = position;
    }

    @Override
    public double getPosition() {
        return this.position;
    }
}
