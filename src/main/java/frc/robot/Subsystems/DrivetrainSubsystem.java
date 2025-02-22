package frc.robot.Subsystems;


import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;

//import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonTrackedTarget;

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
        PhotonCamera camera = new PhotonCamera("photonvision");

        private final PIDController visionForwardBackController = new PIDController(0, 0, 0);
        private final PIDController visionSidewaysController = new PIDController(0, 0, 0);
        private final PIDController visionRotationsController = new PIDController(0, 0, 0);

    //CONSTRUCTOR//
        public DrivetrainSubsystem(swerveModule... modules) {
            IMU = new ADIS16470_IMU();
            //pigeon = new Pigeon2(Constants.PigeonID,"Default Name");
            this.modules = modules;
            kinematics = new SwerveDriveKinematics(Constants.moduleLocations);

            visionForwardBackController.setTolerance(0);
            visionSidewaysController.setTolerance(0);
            visionRotationsController.setTolerance(0);

            setDefaultCommand(
                runOnce(
                        () -> {
                        drive(new ChassisSpeeds(0, 0, 0));
                        })
                    .andThen(run(() -> {}))
                    .withName("Idle"));

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

        public Command executeAimCommand() {
            return parallel(
                run(() -> {
                    // TODO: need to capture on the first iteration, but then continue if interrupted for a certain amount of time before aborting
                    var results = camera.getAllUnreadResults();
                    if (!results.isEmpty()) {
                        var result = results.get(results.size() - 1);
                        if (result.hasTargets()) {
                            var bestTarget = result.getBestTarget();
                            if (bestTarget != null) {
                                var translation = bestTarget.getBestCameraToTarget();
                                var forwardVelocity = MathUtil.clamp(visionForwardBackController.calculate(translation.getX(), 0), -0.1, 0.1);
                                var sidewaysVelocity = MathUtil.clamp(visionSidewaysController.calculate(translation.getY(), 0), -0.1, 0.1);
                                var angularVelcoity = MathUtil.clamp(visionRotationsController.calculate(bestTarget.getYaw(), 0), -0.1, 0.1);

                                var speeds = new ChassisSpeeds(forwardVelocity, sidewaysVelocity, angularVelcoity);
                                drive(speeds);
                            }
                        }
                    }
                }),
                waitUntil(() ->
                    visionForwardBackController.atSetpoint() &&
                        visionSidewaysController.atSetpoint() &&
                        visionRotationsController.atSetpoint()
                )
            );
        }

        public Command executeDriveForwardCommand() {
            return run(() -> {
                driveFieldRelative(new ChassisSpeeds(0.1, 0, 0));
            });
        }
    ////
    /// Camera Data
}