package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import xyz.malefic.frc.pingu.motor.talonfx.TonguFX
import frc.robot.utils.RobotParameters.TransportParameters.INDEXER_MOTOR_PINGU
import frc.robot.utils.RobotParameters.MotorParameters.INDEXER_MOTOR_ID
import frc.robot.utils.RobotParameters.TransportParameters.HOPPER_MOTOR_PINGU
import frc.robot.utils.RobotParameters.MotorParameters.HOPPER_MOTOR_ID
import frc.robot.utils.RobotParameters.TransportParameters.transportState
import frc.robot.utils.emu.TransportState

object Transport : SubsystemBase() {
    private val velocitySetter = VelocityTorqueCurrentFOC(0.0)

    private val indexerMotor =
        TonguFX(INDEXER_MOTOR_ID, velocitySetter, { out -> this.withVelocity(out) }) {
            pingu = INDEXER_MOTOR_PINGU
            neutralMode = NeutralModeValue.Brake
            inverted = InvertedValue.Clockwise_Positive
            name = "Index Motor"
        }

    private val hopperMotor =
        TonguFX(HOPPER_MOTOR_ID, velocitySetter, { out -> this.withVelocity(out) }) {
            pingu = HOPPER_MOTOR_PINGU
            neutralMode = NeutralModeValue.Brake
            inverted = InvertedValue.Clockwise_Positive
            name = "Hopper Motor"
        }

    override fun periodic() {
        setTransportVelocity(transportState.velocity)
    }

    fun setTransportVelocity(speed : Double) {
        hopperMotor.setControl(velocitySetter.withVelocity(speed))
        indexerMotor.setControl(velocitySetter.withVelocity(speed))
    }
}