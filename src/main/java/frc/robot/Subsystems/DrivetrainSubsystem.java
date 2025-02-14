package frc.robot.Subsystems;


import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;

//import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase{

    ////
    ADIS16470_IMU IMU;
    Pigeon2 pigeon;
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
        IMU = new ADIS16470_IMU();
        //pigeon = new Pigeon2(Constants.PigeonID,"Default Name");
        this.modules = modules;
        kinematics = new SwerveDriveKinematics(Constants.moduleLocations);

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds),

            new PPHolonomicDriveController(
                new PIDConstants(Constants.DriveTrain.SpeedKP, Constants.DriveTrain.SpeedKI, Constants.DriveTrain.SpeedKD),
                new PIDConstants(Constants.DriveTrain.SteerKP, Constants.DriveTrain.SteerKI, Constants.DriveTrain.SteerKD)
            ),
            config,
            () -> {
              var alliance = DriverStation.getAlliance();
              if (!alliance.isPresent()) {
                return false;
              }
              return alliance.get() == DriverStation.Alliance.Red;
            },
            this
        );
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
        return Rotation2d.fromDegrees(IMU.getAngle());
        //return Rotation2d.fromDegrees(pigeon.getRoll().getValueAsDouble());
    }

    public void calibrateGyro(){
        IMU.calibrate();
        //pigeon.
    }

    public void resetGyro(){
        IMU.reset();
        //pigeon.
    }

    //POSE//
    private Pose2d getPose() {
        // TODO: implement
        return new Pose2d();
    }

    private void resetPose(Pose2d pose) {
        // TODO: implement
    }

    //DRIVING//
    private ChassisSpeeds getRobotRelativeSpeeds() {
        // TODO: implement
        return new ChassisSpeeds();
    }

    private void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        // TODO: implement, might be same as `drive` or `driveFieldRelative`
    }

    public void drive(ChassisSpeeds chassisSpeeds){
        setModuleTargetStates(chassisSpeeds);
    }

    public void driveFieldRelative(ChassisSpeeds chassisSpeeds){
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