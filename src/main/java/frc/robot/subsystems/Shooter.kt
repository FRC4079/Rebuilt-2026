package frc.robot.subsystems

import com.ctre.phoenix6.controls.MotionMagicVoltage
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.utils.RobotParameters.MotorParameters.SHOOTER_CLOCKWISE_MOTOR_ID
import frc.robot.utils.RobotParameters.MotorParameters.SHOOTER_COUNTER_MOTOR_ID
import frc.robot.utils.RobotParameters.MotorParameters.SHOOTER_HOOD_MOTOR_ID
import frc.robot.utils.RobotParameters.ShooterParameters.COUNTER_PINGU
import frc.robot.utils.RobotParameters.ShooterParameters.CLOCKWISE_PINGU
import frc.robot.utils.RobotParameters.ShooterParameters.HOOD_PINGU
import frc.robot.utils.RobotParameters.ShooterParameters.shooterState
import frc.robot.utils.RobotParameters.ShooterParameters.hoodState
import frc.robot.utils.emu.ShooterState
// import xyz.malefic.frc.pingu.log.LogPingu.log
import xyz.malefic.frc.pingu.motor.talonfx.TonguFX
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.utils.emu.HoodState
import frc.robot.utils.emu.SwerveDriveState
import frc.robot.utils.RobotParameters.SwerveParameters.swerveState


object Shooter : SubsystemBase() {
    private val voltageControl: VelocityTorqueCurrentFOC = VelocityTorqueCurrentFOC(0.0)
    private val positionRequest: MotionMagicVoltage = MotionMagicVoltage(0.0).withEnableFOC(true)

    private val shooterMotorClockwise =
        TonguFX( SHOOTER_CLOCKWISE_MOTOR_ID, voltageControl, { out -> this.withVelocity(out) }) {
            pingu = CLOCKWISE_PINGU
            neutralMode = NeutralModeValue.Brake
            inverted = InvertedValue.Clockwise_Positive
            name = "Shooter Motor Clockwise"
        }

    private val shooterMotorCounter =
        TonguFX( SHOOTER_COUNTER_MOTOR_ID, voltageControl, { out -> this.withVelocity(out) }) {
            pingu = COUNTER_PINGU
            neutralMode = NeutralModeValue.Brake
            inverted = InvertedValue.CounterClockwise_Positive
            name = "Shooter Motor Counter"
        }

    private val hoodMotor =
        TonguFX(SHOOTER_HOOD_MOTOR_ID, positionRequest, { out -> this.withPosition(out) }) {
            pingu = HOOD_PINGU
            neutralMode = NeutralModeValue.Brake
            inverted = InvertedValue.Clockwise_Positive
            name = "Shooter Hood Motor"
        }

    override fun periodic() {

        shooterState = when (swerveState){
            SwerveDriveState.FIELD_ORIENTED -> {ShooterState.OFF}
            SwerveDriveState.SHOOTING -> {ShooterState.FULL_SPEED}
        }

        hoodState = when (swerveState){
            SwerveDriveState.FIELD_ORIENTED -> {HoodState.STOP}
            SwerveDriveState.SHOOTING -> {HoodState.TRACKING}
        }

        aimHoodAtTarget(ShooterCalculator.currentInterceptSolution)
        setShooterSpeed(-shooterState.velocity, shooterState.velocity)
    }

    /**
     * Sets the speed of both shooter motors.
     *
     * @param clockwiseSpeed The speed for the clockwise motor.
     * @param counterSpeed The speed for the counter motor.
     */

    fun setShooterSpeed(clockwiseSpeed: Double, counterSpeed: Double) {
        shooterMotorClockwise.setControl(voltageControl.withVelocity(clockwiseSpeed))
        shooterMotorCounter.setControl(voltageControl.withVelocity(counterSpeed))
    }

//    fun setHoodSpeed(hoodPos: Double) {
//        hoodMotor.setControl(positionRequest.withPosition(hoodSpeed))
//    }

    fun convertInterceptSolutionToPitch(interceptSolution: ShooterCalculator.InterceptSolution) : Double {
        // oough im magic number-ing it im magic number-ing it so good
        val counts : Double = interceptSolution.pitchRad * (4096.0 / (2.0 * Math.PI)) // convert radians to motor counts
        return counts
    }

    // I don't know if this works we supposedly need to convert the pitchRad to a position but this might just be adding/subbing
    fun aimHoodAtTarget(interceptSolution: ShooterCalculator.InterceptSolution) {
        val hoodPositionCounts : Double = convertInterceptSolutionToPitch(interceptSolution)
        hoodMotor.setControl(positionRequest.withPosition(hoodPositionCounts))
    }
}