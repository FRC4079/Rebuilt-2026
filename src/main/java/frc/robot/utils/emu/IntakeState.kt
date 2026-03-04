package frc.robot.utils.emu

/**
 * An enum representing the state of the intake.
 * @property velocity The velocity per state, used to set the motor speed.
 */
enum class IntakeState (
    val velocity: Double,
) {
    /** Represents the intake motor at full power clockwise, intaking. */
    INTAKE(8.0),
    /** Represents the intake motor at full power counter-clockwise, if you for some reason need this. */
    OUTTAKE(-1.0),
    /** Represents the intake motor stopped at 0.0 velocity. */
    STOP(0.0),
}