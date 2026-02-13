package frc.robot.utils

import edu.wpi.first.math.kinematics.ChassisSpeeds

// once again thank you houndutil frc868

// this is kinda shitty but whatever have an empty class for chassis accelerations

/**
 * Data class representing the chassis accelerations of a robot. Can be constructed either by directly setting ax/ay/omega or by providing current and previous chassis speeds along with the time delta.
 *
 * @property axMetersPerSecondsSquared The acceleration in the x direction (forward) in meters per second squared.
 * @property ayMetersPerSecondsSquared The acceleration in the y direction (sideways) in meters per second squared.
 * @property omegaRadiansPerSecondsSquared The angular acceleration in radians per second squared.
 */

data class ChassisAccelerations (
    var axMetersPerSecondsSquared: Double,
    var ayMetersPerSecondsSquared: Double,
    var omegaRadiansPerSecondsSquared: Double){

    constructor(speed: ChassisSpeeds, previousSpeed: ChassisSpeeds, deltaTimeSeconds: Double) : this(
        0.0, 0.0, 0.0
    ) {
        axMetersPerSecondsSquared = (speed.vxMetersPerSecond - previousSpeed.vxMetersPerSecond) / deltaTimeSeconds
        ayMetersPerSecondsSquared = (speed.vyMetersPerSecond - previousSpeed.vyMetersPerSecond) / deltaTimeSeconds
        omegaRadiansPerSecondsSquared = (speed.omegaRadiansPerSecond - previousSpeed.omegaRadiansPerSecond) / deltaTimeSeconds
    }
}