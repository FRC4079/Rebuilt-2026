package frc.robot.utils.emu

/**
 * An enum representing the state of the intake. On/Off.
 * @property velocity The velocity per state, used to set the motor speed.
 */
enum class TransportState (
    val velocity: Double,
) {
    ON(1.0),
    STOP(0.0),
}