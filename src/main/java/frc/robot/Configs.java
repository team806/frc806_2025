package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



public final class Configs {

    public static final class Drive{

        public static final SparkMaxConfig driveConfig = new SparkMaxConfig();
            static{
                 driveConfig.idleMode(IdleMode.kBrake)
                            .smartCurrentLimit(Constants.Drivetrain.driveCurrentLimit)
                            .voltageCompensation(12);
            }
        public static final SparkMaxConfig steerConfig = new SparkMaxConfig();
            static{
                steerConfig.idleMode(IdleMode.kBrake)
                           .smartCurrentLimit(Constants.Drivetrain.steerCurrentLimit)
                           .voltageCompensation(12);
            }
    }

    public static final class elevator{

        public static final SparkMaxConfig leaderConfig = new SparkMaxConfig();
        
            static{
                leaderConfig.idleMode(IdleMode.kBrake)
                            .smartCurrentLimit(Constants.Elevator.CurrentLimit)
                            .voltageCompensation(12);
                leaderConfig.closedLoop //FIXME find values
                            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                            .p(0)
                            .i(0)
                            .d(0);
                
            }
            public static final SparkMaxConfig followerConfig = new SparkMaxConfig();
            static{
                followerConfig.idleMode(IdleMode.kBrake)
                              .smartCurrentLimit(Constants.Elevator.CurrentLimit)
                              .voltageCompensation(12)
                              .inverted(true)
                              .follow(Constants.Elevator.leaderID);
            }
    }

    public static final class arm{

        public static final SparkFlexConfig pivotConfig = new SparkFlexConfig();
            static{
                pivotConfig.idleMode(IdleMode.kBrake)
                           .smartCurrentLimit(Constants.Arm.pivotCurrentLimit)
                           .voltageCompensation(12);
                pivotConfig.closedLoop //FIXME find values
                           .p(0)
                           .i(0)
                           .d(0);
            }
        public static final SparkFlexConfig wheelConfig = new SparkFlexConfig();
            static{
                wheelConfig.idleMode(IdleMode.kBrake)
                           .smartCurrentLimit(Constants.Arm.wheelCurrentLimit)
                           .voltageCompensation(12);
            }
    }

    public static final class intake{

        public static final SparkFlexConfig angConfig = new SparkFlexConfig();
            static{
                angConfig.idleMode(IdleMode.kBrake)
                         .smartCurrentLimit(Constants.Pconstants.angCurrentLimit)
                         .voltageCompensation(12);
                angConfig.encoder
                         .velocityConversionFactor(Constants.Pconstants.velocityConversionFactor);
                angConfig.closedLoop
                         .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                         .p(Constants.Pconstants.ControllerKP)
                         .i(Constants.Pconstants.ControllerKI)
                         .d(Constants.Pconstants.ControllerKD);
            }
        public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
            static{
                intakeConfig.idleMode(IdleMode.kBrake)
                            .smartCurrentLimit(Constants.Pconstants.intakeCurrentLimit)
                            .voltageCompensation(12);
            }
    }

    public static final class climber{

        public static final SparkFlexConfig climbConfig = new SparkFlexConfig();
            static{
                climbConfig.idleMode(IdleMode.kBrake)
                           .smartCurrentLimit(Constants.Climber.CurrentLimit)
                           .voltageCompensation(12);
            }
    }
}
