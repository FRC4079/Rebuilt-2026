package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import xyz.malefic.frc.pingu.motor.talonfx.TonguFX
import frc.robot.utils.RobotParameters.IntakeParameters.INTAKE_MOTOR_PINGU
import frc.robot.utils.RobotParameters.MotorParameters.INTAKE_MOTOR_ID
import frc.robot.utils.RobotParameters.IntakeParameters.intakeState
import frc.robot.utils.RobotParameters.IntakeParameters.intakePivotState
import frc.robot.utils.emu.IntakeState
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.subsystems.Swerve.pidgeyYaw
import frc.robot.utils.RobotParameters.IntakeParameters.INTAKE_MOTOR_PIVOT_PINGU
import frc.robot.utils.RobotParameters.MotorParameters.INTAKE_PIVOT_MOTOR_ID
import frc.robot.utils.emu.IntakePivotState

object Intake : SubsystemBase() {
    private val velocitySetter = VelocityVoltage(0.0)
    private val positionSetter = PositionVoltage(0.0)

    private val intakeMotor =
        TonguFX(INTAKE_MOTOR_ID, velocitySetter, { out -> this.withVelocity(out) }) {
            pingu = INTAKE_MOTOR_PINGU
            neutralMode = NeutralModeValue.Brake
            inverted = InvertedValue.CounterClockwise_Positive
            name = "Intake Motor"
        }
    private val intakePivotMotor =
        TonguFX(INTAKE_PIVOT_MOTOR_ID, positionSetter, { out -> this.withPosition(out) }) {
            pingu = INTAKE_MOTOR_PIVOT_PINGU
            neutralMode = NeutralModeValue.Brake
            inverted = InvertedValue.Clockwise_Positive
            name = "Intake Pivot Motor"
        }

    override fun periodic() {
        setIntakeVelocity(intakeState.velocity)
        when (intakeState) {
            IntakeState.STOP -> movePivot(IntakePivotState.UP) //do nothing
            IntakeState.INTAKE -> movePivot(IntakePivotState.DOWN)
            IntakeState.OUTTAKE -> print("no")
        }

        SmartDashboard.putNumber("Comanded Intake Pivot Velocity", intakePivotMotor.getVelocity().valueAsDouble)
    }

    fun setIntakeVelocity(speed : Double) {
        intakeMotor.setControl(velocitySetter.withVelocity(speed))
    }

    fun movePivot(state: IntakePivotState){
        intakePivotState = state
        intakePivotMotor.setControl(positionSetter.withPosition(state.position).withFeedForward(0.0))
    }
}