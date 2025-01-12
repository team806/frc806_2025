package frc.robot.Subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.hal.HAL;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.IntakeState;
import frc.robot.Constants;
import frc.robot.IO.SparkMaxIO;
import frc.robot.Mocks.MockSparkMaxIO;

public class IntakeTest {
    Intake m_intake;
    MockSparkMaxIO mockAngleMotorIO;

    @BeforeEach
    void Setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        // DCMotor motor = new DCMotor(0.0, 0.0, 0.0, 0.0, 0.0, 1);
        // angleMotorSim = new SparkMaxSim(new SparkMax(Constants.Intake.AngMotorID,
        // MotorType.kBrushless), motor);
        mockAngleMotorIO = new MockSparkMaxIO();
        m_intake = new Intake(mockAngleMotorIO);
    }

    @Test
    void stateTest() {
        // Starts with intake retracted
        mockAngleMotorIO.setPosition(Constants.Intake.retractedSetPoint);
        assertEquals(IntakeState.INTAKE_STATE_RETRACTED, m_intake.Update(false, false, false, false, false));
        mockAngleMotorIO.setPosition(Constants.Intake.ampSetPoint);
        assertEquals(IntakeState.INTAKE_STATE_AMP, m_intake.Update(false, false, false, false, false));
    }
}
