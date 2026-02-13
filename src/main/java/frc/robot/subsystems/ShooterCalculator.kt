package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.Swerve.pose
import frc.robot.utils.RobotParameters.FieldParameters.BLUE_HUB_SCORE_POSITION
import frc.robot.utils.RobotParameters.FieldParameters.RED_HUB_SCORE_POSITION
import frc.robot.utils.RobotParameters.GameParameters.teamColor
import frc.robot.utils.RobotParameters.ShooterParameters.SHOOTER_RPM_SCALING
import frc.robot.utils.RobotParameters.ShooterParameters.SHOOTER_POSE_OFFSET
import frc.robot.utils.RobotParameters.BallParameters.GRAVITY
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.robot.utils.ChassisAccelerations
import java.util.function.Function
import kotlin.math.abs
import kotlin.math.atan
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sqrt

object ShooterCalculator : SubsystemBase() {
    private var targetDistance : Double = 0.0
    private var targetRPS : Double = 0.0

     var currentEffectiveYaw : Double = 0.0
     var currentEffectiveTargetPose = Pose3d.kZero
     var currentInterceptSolution : InterceptSolution = InterceptSolution(
        BLUE_HUB_SCORE_POSITION,
        0.0,
        0.0,
        0.0,
        0.0
    )
    var targetLocation : Pose3d = BLUE_HUB_SCORE_POSITION

    override fun periodic() {
        targetLocation = teamColor.let {
            if (it == "Blue"){
                BLUE_HUB_SCORE_POSITION
            } else {
                RED_HUB_SCORE_POSITION
            }
        }
        targetDistance = pose.translation.getDistance(BLUE_HUB_SCORE_POSITION.toPose2d().translation)
        targetRPS = SHOOTER_RPM_SCALING.get(targetDistance)

        val shooterPose = Pose3d(pose).plus(SHOOTER_POSE_OFFSET)

        val drivetrainSpeeds : ChassisSpeeds = Swerve.fieldRelativeVelocity
        val drivetrainAccelerations : ChassisAccelerations = Swerve.fieldRelativeAccelerations

        currentInterceptSolution = solveShootOnTheFly(
            shooterPose,
            targetLocation,
            drivetrainSpeeds,
            drivetrainAccelerations,
            targetRPS,
            5,
            0.01
        )

        currentEffectiveTargetPose = currentInterceptSolution.effectiveTargetPose
        currentEffectiveYaw = currentInterceptSolution.yawRad
    }

    /**
     * Calculates the time it will take for a projectile to reach the target from the robot's current pose.
     *
     * @param robotPose The current 2dPose of the robot
     * @param targetPose The 3dPose of the target
     * @param xyDistanceToProjectileVelocity A function that uses the xy distance to the target to return the projectile velocity in m/s.
     */
    private fun getTimeToShoot(robotPose: Pose2d, targetPose: Pose3d, xyDistanceToProjectileVelocity: Function<Double, Double>): Double {
        val diff: Transform3d = Pose3d(robotPose).minus(targetPose)
        val xyDistance: Double = Translation2d(diff.x, diff.y).norm
        val distance : Double = diff.translation.norm
        val projectileVelocity: Double = xyDistanceToProjectileVelocity.apply(xyDistance)
        return distance / projectileVelocity
    }

    /**
     * Calculates the displaced target position accounting for robot velocity and acceleration.
     *
     * @param robotPose The current 2dPose of the robot
     * @param targetPose The 3dPose of the target
     * @param xyDistanceToProjectileVelocity A function that uses the xy distance to the target to return the projectile velocity in m/s.
     * @param robotVelocity The current chassis speeds of the robot in field-relative coordinates.
     * @param robotAcceleration The current chassis accelerations of the robot in field-relative coordinates.
     * @param iterations The number of iterations, **higher = more precise**.
     * @param accelerationCompensation A factor to scale the acceleration compensation.
     * @return The adjusted 3dPose of the target accounting for robot motion.
     */
    private fun calculateDisplacedTargetPosition(
        robotPose: Pose2d,
        targetPose: Pose3d,
        xyDistanceToProjectileVelocity: Function<Double, Double>,
        robotVelocity: ChassisSpeeds,
        robotAcceleration: ChassisAccelerations,
        iterations : Int,
        accelerationCompensation : Double
    ) : Pose3d {
        var shotTime : Double = getTimeToShoot(robotPose, targetPose, xyDistanceToProjectileVelocity)

        val correctedTargetPose = Pose3d()

        for (i in iterations - 1 downTo 0 step 1) {
            // Probs inefficient in terms of bytecode, but we have bigger fish to fry
            val virtualGoalX : Double = targetPose.x - shotTime * (robotVelocity.vxMetersPerSecond + robotAcceleration.axMetersPerSecondsSquared * accelerationCompensation)
            val virtualGoalY : Double = targetPose.y - shotTime * (robotVelocity.vyMetersPerSecond + robotAcceleration.ayMetersPerSecondsSquared * accelerationCompensation)
            val correctedTargetPose = Pose3d(virtualGoalX, virtualGoalY, targetPose.z, targetPose.rotation)
            val newShotTime : Double = getTimeToShoot(robotPose, correctedTargetPose, xyDistanceToProjectileVelocity)

            shotTime = newShotTime
            if (abs(newShotTime - shotTime) <= 0.01) {
                break
            }
        }
        return correctedTargetPose
    }

    /**
     * Data class representing a solution for intercepting a moving target.
     *
     * @param effectiveTargetPose The adjusted pose of the target, accounting for robot velocity/accel
     * @param pitchRad The launch pitch angle in radians.
     * @param launchSpeed The required launch speed in meters per second.
     * @param flightTimeSeconds The time of flight in seconds.
     * @param yawRad The launch yaw angle in radians.
     */
    @JvmRecord
    data class InterceptSolution(
        val effectiveTargetPose: Pose3d,
        val pitchRad : Double,
        val launchSpeed : Double,
        val flightTimeSeconds: Double,
        val yawRad : Double,
    )

    /**
     * Data class representing a ballistic shot solution.
     *
     * @param launchPitch The launch pitch angle in radians.
     * @param launchSpeed The required launch speed in meters per second.
     * @param flightTimeSeconds The time of flight in seconds.
     */
    @JvmRecord
    data class ShotSolution(
        val launchPitch: Double,
        val launchSpeed: Double,
        val flightTimeSeconds: Double,
    )

    /**
     * Solves a ballistic equation to find the necessary parts for a [ShotSolution]'s pitch, speed, and time.
     *
     * @param shooterPose3d The 3D pose of the shooter.
     * @param targetPose3d The 3D pose of the target.
     * @param launchSpeed The launch speed of the projectile.
     */
    private fun solveBallisticUsingSpeed(
        shooterPose3d: Pose3d,
        targetPose3d: Pose3d,
        launchSpeed: Double,
    ) : ShotSolution{
        val shootTrans : Translation3d = shooterPose3d.translation
        val targetTrans : Translation3d = targetPose3d.translation

        val dx: Double = targetTrans.x - shootTrans.x
        val dy : Double = targetTrans.y - shootTrans.y
        val dz : Double = targetTrans.z - shootTrans.z

        val horDist : Double = hypot(dx, dy)
        if (horDist <= 1e-9){
            throw IllegalArgumentException("Horizontal distance between shooter and target is too small.")
        }

        val v2 = launchSpeed * launchSpeed

        val discriminant : Double = v2 * v2 - GRAVITY * (GRAVITY * horDist * horDist + 2.0 * dz * v2)
        if (discriminant < 0){
            return ShotSolution(0.0,0.0,0.0)
        }

        val tanTheta : Double = (v2 + sqrt(discriminant)) / (GRAVITY * horDist)
        val launchPitch : Double = atan(tanTheta)
        val time : Double = horDist / (launchSpeed * cos(launchPitch))

        return ShotSolution(launchPitch, launchSpeed, time)
    }

    private fun solveShootOnTheFly(
        shooterPose3d: Pose3d,
        targetPose3d: Pose3d,
        fieldRelRobotVelocity: ChassisSpeeds,
        fieldRelRobotAcceleration: ChassisAccelerations,
        targetRPS : Double,
        iterations : Int,
        timeTolerance : Double,
    ) : InterceptSolution {
        var solution: ShotSolution = solveBallisticUsingSpeed(shooterPose3d, targetPose3d,targetRPS)
        var time : Double = solution.flightTimeSeconds
        var effectiveTargetPose3d : Pose3d = targetPose3d
        var newSolution : ShotSolution

        for (i in iterations - 1 downTo 0 step 1) {
            val dx : Double = fieldRelRobotVelocity.vxMetersPerSecond * time
            val dy : Double = fieldRelRobotVelocity.vyMetersPerSecond * time

            effectiveTargetPose3d = Pose3d(
                targetPose3d.x + dx,
                targetPose3d.y + dy,
                targetPose3d.z,
                targetPose3d.rotation
            )

            newSolution = solveBallisticUsingSpeed(shooterPose3d, effectiveTargetPose3d, targetRPS)

            if(abs(newSolution.flightTimeSeconds - time) < timeTolerance) {
                return InterceptSolution(
                    effectiveTargetPose3d,
                    newSolution.launchPitch,
                    newSolution.launchSpeed,
                    newSolution.flightTimeSeconds,
                    0.0
                )
            }

            solution = newSolution
            time = newSolution.flightTimeSeconds
        }

        return InterceptSolution(
            effectiveTargetPose3d,
            solution.launchPitch,
            solution.launchSpeed,
            solution.flightTimeSeconds,
            0.0
        )
    }
}