package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;



public class Elevator extends SubsystemBase {
//   // Hardware interface and inputs
//   private final Elevator io;
//   private final ElevatorIOInputsAutoLogged inputs;

//   // Current arm distance mode
//   private ElevatorPosition currentMode = ElevatorPosition.INTAKE;

//   // Alerts for motor connection status
//   private final Alert leaderMotorAlert =
//       new Alert("Elevator leader motor isn't connected", AlertType.kError);
//   private final Alert followerMotorAlert =
//       new Alert("Elevator follower motor isn't connected", AlertType.kError);
    private final SparkMax liftMotor;
    private final SparkMax followerMotor;
    private final RelativeEncoder liftEncoder;
    
    
        public Elevator() {
            liftMotor = new SparkMax(Constants.Elevator.leaderID, MotorType.kBrushless);
                liftMotor.configure(Configs.elevator.leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            followerMotor = new SparkMax(Constants.Elevator.followerID, MotorType.kBrushless);
                followerMotor.configure(Configs.elevator.followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        liftEncoder = liftMotor.getEncoder();
    }

    public void periodic() {
        SmartDashboard.putNumber("elevator lift", liftEncoder.getPosition());
    }

//   @Override
//   public void periodic() {
//     // Update and log inputs from hardware
//     io.updateInputs(inputs);
//         Logger.processInputs("Elevator", inputs);
    
//         // Update motor connection status alerts
//         leaderMotorAlert.set(!inputs.leaderConnected);
//         followerMotorAlert.set(!inputs.followerConnected);
//       }
    
//       private void updateInputs(ElevatorIOInputsAutoLogged inputs2) {
//         // TODO Auto-generated method stub
//         throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
//       }
    
//       /**
//    * Runs the arm in closed-loop distance mode to the specified angle.
//    *
//    * @param distance The target angle distance
//    */
//   private void setDistance(Distance distance) {
//     io.setDistance(distance);
//   }

//   public void setManual(double power) {
//     io.setManual(power);
//   }

//   public void stopHere() {
//     io.stopHere();
//   }

//   /** Stops the arm motors. */
//   private void stop() {
//     io.stop();
//   }

//   /**
//    * Returns the current distance of the arm.
//    *
//    * @return The current angular distance
//    */
//   @AutoLogOutput
//   public Distance getPosition() {
//     return inputs.elevatorDistance;
//   }

//   /** Enumeration of available arm distances with their corresponding target angles. */
//   private enum ElevatorPosition {
//     STOP(Inches.of(0)), // Stop the arm
//     INTAKE(Inches.of(0), Inches.of(.5)), // Elevator tucked in
//     L1(Inches.of(12), Inches.of(.5)), // Position for scoring in L1
//     L2(Inches.of(15.75), Inches.of(.5)), // Position for scoring in L2
//     L3(Inches.of(30.25), Inches.of(.5)), // Position for scoring in L3
//     L4(Inches.of(55), Inches.of(.5)), // Position for scoring in L4
//     ALGAE_LOW(Inches.of(10), Inches.of(1)), // Position for grabbing low algae
//     ALGAE_HIGH(Inches.of(25), Inches.of(1)); // Position for grabbing high algae

//     private final Distance targetDistance;
//     private final Distance angleTolerance;

//     ElevatorPosition(Distance targetDistance, Distance angleTolerance) {
//       this.targetDistance = targetDistance;
//       this.angleTolerance = angleTolerance;
//     }

//     ElevatorPosition(Distance targetDistance) {
//       this(targetDistance, Inches.of(2)); // 2 degree default tolerance
//     }
//   }

//   /**
//    * Gets the current arm distance mode.
//    *
//    * @return The current ElevatorPosition
//    */
//   public ElevatorPosition getMode() {
//     return currentMode;
//   }

//   /**
//    * Sets a new arm distance and schedules the corresponding command.
//    *
//    * @param mode The desired ElevatorPosition
//    */
//   private void setElevatorPosition(ElevatorPosition mode) {
//     if (currentMode != mode) {
//       currentCommand.cancel();
//       currentMode = mode;
//       currentCommand.schedule();
//     }
//   }

//   // Command that runs the appropriate routine based on the current distance
//   private final Command currentCommand =
//       new SelectCommand<>(
//           Map.of(
//               ElevatorPosition.STOP,
//               Commands.runOnce(this::stop).withName("Stop Elevator"),
//               ElevatorPosition.INTAKE,
//               createPositionCommand(ElevatorPosition.INTAKE),
//               ElevatorPosition.L1,
//               createPositionCommand(ElevatorPosition.L1),
//               ElevatorPosition.L2,
//               createPositionCommand(ElevatorPosition.L2),
//               ElevatorPosition.L3,
//               createPositionCommand(ElevatorPosition.L3),
//               ElevatorPosition.L4,
//               createPositionCommand(ElevatorPosition.L4),
//               ElevatorPosition.ALGAE_LOW,
//               createPositionCommand(ElevatorPosition.ALGAE_LOW),
//               ElevatorPosition.ALGAE_HIGH,
//               createPositionCommand(ElevatorPosition.ALGAE_HIGH)),
//           this::getMode);

//   /**
//    * Creates a command for a specific arm distance that moves the arm and checks the target
//    * distance.
//    *
//    * @param distance The arm distance to create a command for
//    * @return A command that implements the arm movement
//    */
//   private Command createPositionCommand(ElevatorPosition distance) {
//     return Commands.runOnce(() -> setDistance(distance.targetDistance))
//         .withName("Move to " + distance.toString());
//   }

//   /**
//    * Checks if the arm is at its target distance.
//    *
//    * @return true if at target distance, false otherwise
//    */
//   @AutoLogOutput
//   public boolean isAtTarget() {
//     if (currentMode == ElevatorPosition.STOP) return true;
//     return getPosition().isNear(currentMode.targetDistance, currentMode.angleTolerance);
//   }

//   /**
//    * Logs target angle for given mode.
//    *
//    * @return The target angle for the current mode
//    */
//   @AutoLogOutput
//   private Distance targetDistance() {
//     return currentMode.targetDistance;
//   }

//   /**
//    * Creates a command to set the arm to a specific distance.
//    *
//    * @param distance The desired arm distance
//    * @return Command to set the distance
//    */
//   private Command setPositionCommand(ElevatorPosition distance) {
//     return Commands.runOnce(() -> setElevatorPosition(distance))
//         .withName("SetElevatorPosition(" + distance.toString() + ")");
//   }

//   /** Factory methods for common distance commands */

//   /**
//    * @return Command to move the arm to L1 scoring distance
//    */
//   public final Command L1() {
//     return setPositionCommand(ElevatorPosition.L1);
//   }

//   /**
//    * @return Command to move the arm to L2 scoring distance
//    */
//   public final Command L2() {
//     return setPositionCommand(ElevatorPosition.L2);
//   }

//   /**
//    * @return Command to move the arm to L3 distance
//    */
//   public final Command L3() {
//     return setPositionCommand(ElevatorPosition.L3);
//   }

//   /**
//    * @return Command to move the arm to L4 distance
//    */
//   public final Command L4() {
//     return setPositionCommand(ElevatorPosition.L4);
//   }

//   /**
//    * @return Command to move the arm to the low algae distance
//    */
//   public final Command AlgaeLow() {
//     return setPositionCommand(ElevatorPosition.ALGAE_LOW);
//   }

//   /**
//    * @return Command to move the arm to the high algae distance
//    */
//   public final Command AlgaeHigh() {
//     return setPositionCommand(ElevatorPosition.ALGAE_HIGH);
//   }

//   /**
//    * @return Command to intake the arm
//    */
//   public final Command intake() {
//     return setPositionCommand(ElevatorPosition.INTAKE);
//   }

//   /**
//    * @return Command to stop the arm
//    */
//   public final Command stopCommand() {
//     return setPositionCommand(ElevatorPosition.STOP);
//   }
}
