package frc.robot.utils.emu

/**
 * An enum representing the state of the intake.
 * @property position Position per state, ahhauhahh.
 */
enum class IntakePivotState (
    val position: Double,
) {
    /** Represents the intake motor at full power clockwise, intaking. */
    DOWN(10.0),
    /** Represents the intake motor at full power counter-clockwise, if you for some reason need this. */
    UP(0.0),
}