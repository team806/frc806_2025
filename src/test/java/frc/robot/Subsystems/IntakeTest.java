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
import frc.robot.Subsystems.Processor;
import frc.robot.Subsystems.ProcessorState;
import frc.robot.Constants;
import frc.robot.IO.SparkMaxIO;
import frc.robot.Mocks.MockSparkMaxIO;

public class IntakeTest {
    Processor m_intake;
    MockSparkMaxIO mockAngleMotorIO;
    MockSparkMaxIO mockIntakeMotorIO;

    @BeforeEach
    void Setup() {
        assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
        // DCMotor motor = new DCMotor(0.0, 0.0, 0.0, 0.0, 0.0, 1);
        // angleMotorSim = new SparkMaxSim(new SparkMax(Constants.Intake.AngMotorID,
        // MotorType.kBrushless), motor);
        mockAngleMotorIO = new MockSparkMaxIO();
        mockIntakeMotorIO = new MockSparkMaxIO();
        m_intake = new Processor(mockAngleMotorIO, mockIntakeMotorIO);
    }

    @Test
    void stateTest() {
        // Starts with intake retracted
        mockAngleMotorIO.setPosition(Constants.Intake.retractedSetPoint);
        assertEquals(ProcessorState.STATE_RETRACTED, m_intake.Update(false, false, false, false, false, 0.0, 0.0, 0, 0));
        mockAngleMotorIO.setPosition(0.3);
        assertEquals(ProcessorState.STATE_AMP, m_intake.Update(false, false, false, false, false, 0.0, 0.0, 0, 0));
    }
}
