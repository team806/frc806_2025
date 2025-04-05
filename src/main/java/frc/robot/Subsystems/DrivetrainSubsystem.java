package frc.robot.Subsystems;


import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DrivetrainSubsystem extends SubsystemBase{

    ////
        ADIS16470_IMU IMU;
        //Pigeon2 IMU;
        public swerveModule[] modules;
        SwerveDriveKinematics kinematics;
        SwerveDriveOdometry odometry;
        ChassisSpeeds m_chassisSpeeds;
        double translationMaxAccelerationMetersPerSecondSquared = 25;
        double rotationMaxAccelerationRadiansPerSecondSquared = 50;
        SlewRateLimiter translationXLimiter = new SlewRateLimiter(translationMaxAccelerationMetersPerSecondSquared);
        SlewRateLimiter translationYLimiter = new SlewRateLimiter(translationMaxAccelerationMetersPerSecondSquared);
        SlewRateLimiter rotationLimiter = new SlewRateLimiter(rotationMaxAccelerationRadiansPerSecondSquared);
        private final StructArrayPublisher<SwerveModuleState> statePublisher;
    //CONSTRUCTOR//
        public DrivetrainSubsystem(swerveModule[] modules) {
            IMU = new ADIS16470_IMU();
            //IMU = new Pigeon2(Constants.PigeonID,"Default Name");
            this.modules = modules;
            kinematics = new SwerveDriveKinematics(Constants.moduleLocations);
            odometry = new SwerveDriveOdometry(kinematics, getGyroscopeRotation(), getModulePositions());
            statePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveModules", SwerveModuleState.struct).publish();
        }
    //GYRO//
        public Rotation2d getGyroscopeRotation() {
            //return Rotation2d.fromDegrees(IMU.get());
        
            return Rotation2d.fromDegrees(-IMU.getAngle(IMUAxis.kY));
            // r*eturn Rotation2d.fromDegrees(IMU.getAngle());
        }
        


     public void calibrateGyro(){
            IMU.calibrate();   
     }
            
        //    IMU.calibrate,
        
        //}

        public void resetGyro(){
            IMU.reset();
            
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
        @Override
        public void periodic() {
            odometry.update(getGyroscopeRotation(), getModulePositions());
            statePublisher.set(new SwerveModuleState[]{
                modules[0].getSwerveModuleState(),
                modules[1].getSwerveModuleState(),
                modules[2].getSwerveModuleState(),
                modules[3].getSwerveModuleState()
            });
        }
        
        public Command getAutonomousCommand() {
            //return m_chooser.getSelected();
            return Commands.deadline(
              waitSeconds(1.5),
              Commands.run(() -> {  drive(new ChassisSpeeds(3, 0, 0)); })
            )
            .andThen(() -> { drive(new ChassisSpeeds(0, 0, 0)); }).withName("Auton");
          }
}