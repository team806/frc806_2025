package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Subsystems.swerveModule;

public class Constants {

    public static final double Delta = 1e-2;
    
    public static final Translation2d[] moduleLocations = {
        new Translation2d(-0.29845,0.29845),  //front right ++
        new Translation2d(-0.29845,-0.29845), //front left  -+
        new Translation2d(0.29845,-0.29845),  //rear left   --
        new Translation2d(0.29845,0.29845)  //rear right  +-
    };
    //velocity constranints for swerve desaturate
    public static final double DriveBaseRadius = 0.42207203769;
    public static final double attainableMaxModuleSpeedMPS = 4.572;
    public static final double attainableMaxTranslationalSpeedMPS = attainableMaxModuleSpeedMPS;
    public static final double attainableMaxRotationalVelocityRPS = attainableMaxModuleSpeedMPS/DriveBaseRadius;

    public static final int PigeonID = 22;   

    public static double controllerDeadband = 0.15; 

    public interface Modules{
            public static final double SpeedKP = 0.001, SpeedKI = 0, SpeedKD = 0.0005;
            public static final double SteerKP = 1.5, SteerKI = 0, SteerKD = 0;
        
            public static final int FrontLeftDriveID   = 4, FrontLeftSteerID   = 5, FrontLeftEncoderID = 6;
			public static final double FrontLeftEncoderOffset = -0.456;//-0.423340 rotations raw = 0.000000 rotations
            // public static final double FrontLeftEncoderOffset = 0;

            public static final int FrontRightDriveID   = 1, FrontRightSteerID   = 2, FrontRightEncoderID = 3;
            public static final double FrontRightEncoderOffset = -0.347;//0.484131 rotations raw = -0.000244 rotations
            // public static final double FrontRightEncoderOffset = 0;

            public static final int RearLeftDriveID   = 7, RearLeftSteerID   = 8, RearLeftEncoderID = 9;
            public static final double RearLeftEncoderOffset = 0.386;//0.283691 rotations raw = -0.000244 rotations
            // public static final double RearLeftEncoderOffset = 0;

            public static final int RearRightDriveID   = 10, RearRightSteerID   = 11, RearRightEncoderID = 12;
            public static final double RearRightEncoderOffset = 0.131;//0.448730 rotations raw = 0.000244 rotations
            // public static final double RearRightEncoderOffset = 0;

        swerveModule[] moduleArray = new swerveModule[] {
            new swerveModule(FrontRightDriveID,FrontRightSteerID,FrontRightEncoderID,FrontRightEncoderOffset),
            new swerveModule(FrontLeftDriveID, FrontLeftSteerID, FrontLeftEncoderID, FrontLeftEncoderOffset),
            new swerveModule(RearLeftDriveID, RearLeftSteerID, RearLeftEncoderID, RearLeftEncoderOffset),
            new swerveModule(RearRightDriveID, RearRightSteerID, RearRightEncoderID, RearRightEncoderOffset)
        };
        
    }
                //TODO not really a todo, but i had to name it this way or java got mad at me
    public interface Pconstants{ //TODO figure out the real values for this section 

        public static final double IntakeSpeed = 0;
        public static final double ShootSpeed = 0;
        

        public static final int AngMotorID = 16, ShootMotorID = 15, PingChannel = 0, EchoChannel = 1;

        public static final double ControllerTolerance = 1;//degrees 
        public static final double ControllerKP = 0.02, ControllerKI = 0, ControllerKD = 0;

        //public static final double DistanceSensorThreshold = 0;

        public static final double scoreAng = 25;
        public static final double ExtendedAngle = 0.78;
        public static final double retractedAngle = 0.95;
        public static final double transportAngle = 0.9;
        public static int angID = 13;
        public static int intakeID = 14;
        public static int algaeSensorPort = 0;

    }

    public interface Drivetrain {
        public static final double TranslationPow = 3;
        public static final double RotationPow = 3;

        public static final double SlowFactor = 3;
        public static final double SlowFactorOffset = 1;
    }

    public interface Shooter{

        public static final int aID = 13,bID = 14;
		public static final double MaxSpeed = 1.00;//percent
        
    }

    public interface Climber{
        public static final int MotorID = 19;
        
        public static final int CurrentLimit = 80;

        public static final double ClimbSpeed = -1.0;
        public static final double ReleaseSpeed = 1.0;
        public static final double BrakeSpeed = 0.0;
    }

    public interface Elevator {

        public interface Lift {
            public static final int MotorID = 15;
            public static final double kFastP = .16/2000;
            public static final double kFastI = .065/2000;
            public static final double kFastD = 0.0025/2000;
            public static final double kSlowP = .16/2000;
            public static final double kSlowI = 0.655/2000;
            public static final double kSlowD = 0.003/2000;

            public static final double IdlePosition = 0;
            public static final double IntakePosition = 650;
            public static final double L1PrepPosition = 2850;
            public static final double A1PrepPosition = 2;
            public static final double L2PrepPosition = 4850; // 7
            public static final double A2PrepPosition = 0;
            public static final double L3PrepPosition = 6850;
            public static final double L4FastPrepPosition = 0;
            public static final double L4PrepPosition = 7130;
            public static final double L4ReleasePosition = 0;
            
        }


        public static interface Arm {
            public static final int MotorID = 18;
            public static final double kP = 1.75;
            public static final double kI = 0;
            public static final double kD = 0.0;

            public static final double IdlePosition = 0.7;
            public static final double IntakePosition = 0.74;
            public static final double L1PrepPosition = 0.4;
            public static final double A1PrepPosition = 0.7;
            public static final double L2PrepPosition = .42; // .4
            public static final double A2PrepPosition = 0;
            public static final double L3PrepPosition = 0.43;
            public static final double L4PrepPosition = .65;

            public static final double A1ReleasePosition = 0;
            public static final double A2ReleasePosition = 0;
        }

        public interface  Intake{
            public static final int MotorID = 17;
            public static final double IntakeSpeed = 0.3;
            public static final double IntakeTimeout = 10;
            public static final double HoldSpeed = 0;
            public static final int sensorPort = 1;
            public static final double ReleaseSpeed = -0.3;
            public static final double AlgaeSpeed = 0;
            public static final double ReleaseTime = 1;
        }

    
    }

    public interface Motion {
            public static final double translationKP = 0.02, translationKI = 0, translationKD = 0;
            public static final double rotationKP = 0.02, rotationKI = 0, rotationKD = 0;
    }

}


