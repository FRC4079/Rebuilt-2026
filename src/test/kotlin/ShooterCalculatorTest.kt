import org.junit.jupiter.api.Assertions.assertEquals

import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import frc.robot.subsystems.ShooterCalculator
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.robot.utils.ChassisAccelerations
import edu.wpi.first.hal.HAL
import frc.robot.utils.RobotParameters

/**
 * Test class for ShooterCalculator.
 *
 * This class contains unit tests for the ShooterCalculator's solveShootOnTheFly method.
 * It verifies that the method correctly calculates the intercept solution for shooting on the fly.
 * Runs on beginning of build
 *
 * @returns Boolean indicating whether the test passed or failed
 */

class ShooterCalculatorTest {

    val shooterCalculatorObject = ShooterCalculator

    @BeforeEach
    fun setUp() {
        // Set up any necessary state before each test
        assert (HAL.initialize(500, 0)) { "HAL initialization failed" }
    }

    @Test
    fun solveShootOnTheFlyTest() {
        val shooterPose = Pose3d(0.0, 0.0, 1.0, Rotation3d.kZero)
        val targetPose = RobotParameters.FieldParameters.RED_HUB_SCORE_POSITION
        val fieldRelRobotVel = ChassisSpeeds(0.0, 0.0, 0.0)
        val fieldRelRobotAccel = ChassisAccelerations(0.0, 0.0, 0.0)
        val targetRPS = 25.0

        val expectedResult = ShooterCalculator.InterceptSolution(
            targetPose,
            1.3,
            25.0,
            3.5,
            0.0
        )

        val result = ShooterCalculator.solveShootOnTheFly(shooterPose, targetPose, fieldRelRobotVel, fieldRelRobotAccel, targetRPS, 5, 0.01)
        assertEquals(expectedResult, result)
    }

}
