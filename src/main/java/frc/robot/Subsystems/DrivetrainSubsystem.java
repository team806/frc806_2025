package frc.robot.Subsystems;


import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase{

    ////
        //ADIS16470_IMU IMU;
        Pigeon2 IMU;
        public swerveModule[] modules;
        SwerveDriveKinematics kinematics;
        //SwerveDriveOdometry odometry;
        ChassisSpeeds m_chassisSpeeds;
        double translationMaxAccelerationMetersPerSecondSquared = 25;
        double rotationMaxAccelerationRadiansPerSecondSquared = 50;
        SlewRateLimiter translationXLimiter = new SlewRateLimiter(translationMaxAccelerationMetersPerSecondSquared);
        SlewRateLimiter translationYLimiter = new SlewRateLimiter(translationMaxAccelerationMetersPerSecondSquared);
        SlewRateLimiter rotationLimiter = new SlewRateLimiter(rotationMaxAccelerationRadiansPerSecondSquared);
    //CONSTRUCTOR//
        public DrivetrainSubsystem(swerveModule... modules) {
            //IMU = new ADIS16470_IMU();
            IMU = new Pigeon2(Constants.PigeonID,"Default Name");
            this.modules = modules;
            kinematics = new SwerveDriveKinematics(Constants.moduleLocations);
        }
    //SINGLETON//
        static DrivetrainSubsystem instance = new DrivetrainSubsystem(
            new swerveModule(Constants.Modules.FrontRightDriveID,Constants.Modules.FrontRightSteerID,Constants.Modules.FrontRightEncoderID,Constants.Modules.FrontRightEncoderOffset),
            new swerveModule(Constants.Modules.FrontLeftDriveID, Constants.Modules.FrontLeftSteerID, Constants.Modules.FrontLeftEncoderID, Constants.Modules.FrontLeftEncoderOffset ),
            new swerveModule(Constants.Modules.RearLeftDriveID,  Constants.Modules.RearLeftSteerID,  Constants.Modules.RearLeftEncoderID,  Constants.Modules.RearLeftEncoderOffset  ),
            new swerveModule(Constants.Modules.RearRightDriveID, Constants.Modules.RearRightSteerID, Constants.Modules.RearRightEncoderID, Constants.Modules.RearRightEncoderOffset )
        );
        public static DrivetrainSubsystem getInstance() {return instance;}
    //GYRO//
        public Rotation2d getGyroscopeRotation() {
            //return Rotation2d.fromDegrees(IMU.get());
            return Rotation2d.fromDegrees(IMU.getYaw().getValueAsDouble());
        }
        


        //public void calibrateGyro(){
        //    IMU.calibra;
        //    //IMU.set
        //}

        public void resetGyro(){
            IMU.reset();
            //pigeon.
        }
    //DRIVING//
        public void drive(ChassisSpeeds  chassisSpeeds){
            setModuleTargetStates(chassisSpeeds);
        }

        public void driveFieldRelative(ChassisSpeeds  chassisSpeeds){
            chassisSpeeds.vxMetersPerSecond = translationXLimiter.calculate(chassisSpeeds.vxMetersPerSecond);
            chassisSpeeds.vyMetersPerSecond = translationYLimiter.calculate(chassisSpeeds.vyMetersPerSecond);
            chassisSpeeds.omegaRadiansPerSecond = rotationLimiter.calculate(chassisSpeeds.omegaRadiansPerSecond);


            setModuleTargetStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroscopeRotation()));
        }

        public void setModuleTargetStates(ChassisSpeeds chassisSpeeds) {
            SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(chassisSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.attainableMaxModuleSpeedMPS);
            modules[0].setTargetState(SwerveModuleState.optimize(targetStates[0], Rotation2d.fromRotations(modules[0].getModuleAngRotations())));
            modules[1].setTargetState(SwerveModuleState.optimize(targetStates[1], Rotation2d.fromRotations(modules[1].getModuleAngRotations())));
            modules[2].setTargetState(SwerveModuleState.optimize(targetStates[2], Rotation2d.fromRotations(modules[2].getModuleAngRotations())));
            modules[3].setTargetState(SwerveModuleState.optimize(targetStates[3], Rotation2d.fromRotations(modules[3].getModuleAngRotations())));

        }
    //FEEDBACK//
        public SwerveModulePosition[] getModulePositions(){
            return new SwerveModulePosition[] {modules[0].getModulePosition(),modules[1].getModulePosition(),modules[2].getModulePosition(),modules[3].getModulePosition()};
        }

        public ChassisSpeeds getChasisSpeed(){
            return kinematics.toChassisSpeeds(
                modules[0].getSwerveModuleState(),
                modules[1].getSwerveModuleState(),
                modules[2].getSwerveModuleState(),
                modules[3].getSwerveModuleState()
            );
        }
    ////
}