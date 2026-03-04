package frc.robot.utils.emu

enum class ShooterState(
    var velocity: Double,
) {
    OFF(0.0),
    BUFFER(0.0),
    FULL_SPEED(-6.0),
    REVERSE(6.0),
}

enum class HoodState() {
    /**
     * Should be unneeded when we implement auto-aim? | changes trajectory to be more upwards.
     */
    STOP,
    TRACKING
}