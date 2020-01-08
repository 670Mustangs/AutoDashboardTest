package org.ghrobotics.falcondashboard.generator.charts.hardcodedvisualizer.paths

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.*
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.mirror
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.derivedunits.*
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.meter
import org.ghrobotics.lib.mathematics.units.meter

object TrajectoryFactory {

    /** Constraints **/

    private val kMaxVelocity = 3.6576.meter.velocity
    private val kMaxAcceleration = 1.8288.meter.acceleration

    private val kMaxHabitatVelocity = 0.9144.meter.velocity

    private val kFirstPathMaxAcceleration = 1.8288.meter.acceleration

    private val kVelocityRadiusConstraintRadius = 0.9144.meter
    private val kVelocityRadiusConstraintVelocity = 0.9144.meter.velocity

    private val kMaxCentripetalAccelerationElevatorUp = 1.8288.meter.acceleration
    private val kMaxCentripetalAccelerationElevatorDown = 2.7432.meter.acceleration

    private val kMaxVoltage = 10.volt

    /** Adjusted Poses **/

    private val cargoShipFLAdjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kCargoShipFL,
        transform = Constants.kForwardIntakeToCenter
    )
    private val cargoShipFRAdjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kCargoShipFR,
        transform = Constants.kForwardIntakeToCenter,
        translationalOffset = Translation2d(0.meter, 0.127.meter)
    )
    private val cargoShipS1Adjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kCargoShipS1,
        transform = Constants.kForwardIntakeToCenter,
        translationalOffset = Translation2d(0.04826.meter, 0.meter)
    )
    private val cargoShipS2Adjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kCargoShipS2,
        transform = Constants.kForwardIntakeToCenter,
        translationalOffset = Translation2d(0.04826.meter, 1.5.meter)
    )
    private val cargoShipS3Adjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kCargoShipS3,
        transform = Constants.kForwardIntakeToCenter
    )
    private val depotAdjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kDepotBRCorner,
        transform = Constants.kBackwardIntakeToCenter
    )
    private val loadingStationAdjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kLoadingStation,
        transform = Constants.kBackwardIntakeToCenter,
        translationalOffset = Translation2d((-0.2286).meter, 0.meter)
    )
    val rocketFAdjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kRocketF,
        transform = Constants.kForwardIntakeToCenter.transformBy(Pose2d(-0.0254.meter, 0.meter)),
        translationalOffset = Translation2d(0.meter, -0.1016.meter)
    )
    private val rocketNAdjusted = TrajectoryWaypoints.Waypoint(
        trueLocation = TrajectoryWaypoints.kRocketN,
        transform = Constants.kForwardIntakeToCenter.transformBy(Pose2d(0.1016.meter, 0.meter))
    )

    /** Trajectories **/

    val cargoShipFLToRightLoadingStation = generateTrajectory(
        true,
        listOf(
            cargoShipFLAdjusted,
            cargoShipFLAdjusted.position.transformBy(Pose2d((-0.21336).meter, 0.meter)).asWaypoint(),
            Pose2d(3.23088.meter, 2.0159472.meter, 69.degree).asWaypoint(),
            loadingStationAdjusted
        ),
        getConstraints(false, loadingStationAdjusted), 2.4384.meter.velocity, 1.8288.meter.acceleration, kMaxVoltage
    )

    val cargoShipFLToLeftLoadingStation = generateTrajectory(
        true,
        listOf(
            cargoShipFLAdjusted,
            cargoShipFLAdjusted.position.transformBy(Pose2d((-0.7).meter, 0.meter)).asWaypoint(),
            Pose2d(3.23088.meter, 2.0159472.meter, 69.degree).mirror.asWaypoint(),
            loadingStationAdjusted.position.mirror.asWaypoint()
        ),
        getConstraints(false, loadingStationAdjusted), 2.4384.meter.velocity, 1.8288.meter.acceleration, kMaxVoltage
    )

    val cargoShipFRToRightLoadingStation = cargoShipFLToLeftLoadingStation.mirror()

    val cargoShipS1ToDepot = generateTrajectory(
        true,
        listOf(
            cargoShipS1Adjusted,
            Pose2d(4.572.meter, 1.5090648.meter, 17.degree).asWaypoint(),
            depotAdjusted
        ),
        getConstraints(false, depotAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val cargoShipS1ToLoadingStation = generateTrajectory(
        true,
        listOf(
            cargoShipS1Adjusted,
            Pose2d(4.572.meter, 1.5090648.meter, 17.degree).asWaypoint(),
            loadingStationAdjusted
        ),
        getConstraints(false, loadingStationAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val centerStartToCargoShipFL = generateTrajectory(
        false,
        listOf(
            TrajectoryWaypoints.kCenterStart.asWaypoint(),
            cargoShipFLAdjusted
        ),
        getConstraints(false, cargoShipFLAdjusted), kMaxVelocity, 1.2192.meter.acceleration, kMaxVoltage
    )

    val centerStartToCargoShipFR = centerStartToCargoShipFL.mirror()

    val depotToCargoShipS2 = generateTrajectory(
        false,
        listOf(
            depotAdjusted,
            Pose2d(4.572.meter, 1.5090648.meter, 17.degree).asWaypoint(),
            cargoShipS2Adjusted
        ),
        getConstraints(false, cargoShipS2Adjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val loadingStationToCargoShipFR = generateTrajectory(
        false,
        listOf(
            loadingStationAdjusted,
            Pose2d(3.23088.meter, 2.0159472.meter, 69.degree).asWaypoint(),
            cargoShipFRAdjusted.position.transformBy(Pose2d((-30).meter, 0.meter)).asWaypoint(),
            cargoShipFRAdjusted
        ),
        getConstraints(false, cargoShipFRAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val loadingStationToCargoShipS2 = generateTrajectory(
        false,
        listOf(
            loadingStationAdjusted,
            Pose2d(4.572.meter, 1.5090648.meter, 17.degree).asWaypoint(),
            cargoShipS2Adjusted
        ),
        getConstraints(false, cargoShipS2Adjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val loadingStationToRocketF = generateTrajectory(
        false,
        listOf(
            loadingStationAdjusted,
            Pose2d(5.1934872.meter, 2.0537424.meter, 9.degree).asWaypoint(),
            rocketFAdjusted
        ),
        getConstraints(true, rocketFAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val loadingStationToRocketN = generateTrajectory(
        false,
        listOf(
            loadingStationAdjusted,
            rocketNAdjusted
        ),
        getConstraints(true, rocketNAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val rocketNToDepot = generateTrajectory(
        true,
        listOf(
            rocketNAdjusted,
            depotAdjusted
        ),
        getConstraints(false, depotAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val rocketFPrepareToRocketF = generateTrajectory(
        false,
        listOf(
            Pose2d(24.074.meter, 3.753.meter, -143.degree).asWaypoint(),
            rocketFAdjusted.position.transformBy(Pose2d(Translation2d(-4.meter, 0.meter))).asWaypoint()
        ),
        getConstraints(false, Pose2d()), 3.meter.velocity, kMaxAcceleration, kMaxVoltage
    )

    val rocketFToDepot = generateTrajectory(
        true,
        listOf(
            rocketFAdjusted,
            Pose2d(19.216.meter, 5.345.meter, 5.degree).asWaypoint(),
            depotAdjusted
        ),
        getConstraints(false, depotAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val rocketFToLoadingStation = generateTrajectory(
        true,
        listOf(
            rocketFAdjusted,
            Pose2d(19.216.meter, 5.345.meter, 5.degree).asWaypoint(),
            loadingStationAdjusted
        ),
        getConstraints(false, loadingStationAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val rocketNToLoadingStation = generateTrajectory(
        true,
        listOf(
            rocketNAdjusted,
            loadingStationAdjusted
        ),
        getConstraints(false, loadingStationAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val sideStartToCargoShipS1 = generateTrajectory(
        false,
        listOf(
            TrajectoryWaypoints.kSideStart.asWaypoint(),
            cargoShipS1Adjusted
        ),
        getConstraints(true, cargoShipS1Adjusted), kMaxVelocity, kFirstPathMaxAcceleration, kMaxVoltage
    )

    val sideStartToRocketF = generateTrajectory(
        false,
        listOf(
            Pose2d(TrajectoryWaypoints.kSideStart.translation).asWaypoint(),
            rocketFAdjusted
        ),
        getConstraints(false, rocketFAdjusted), kMaxVelocity, kMaxAcceleration, kMaxVoltage
    )

    val sideStartReversedToRocketFPrepare = generateTrajectory(
        true,
        listOf(
            TrajectoryWaypoints.kSideStartReversed.asWaypoint(),
            Pose2d(15.214.meter, 8.7.meter, 165.degree).asWaypoint(),
            Pose2d(22.488.meter, 5.639.meter, 143.degree).asWaypoint(),
            Pose2d(24.074.meter, 3.753.meter, -143.degree).asWaypoint()
        ),
        getConstraints(false, Pose2d()), kMaxVelocity, 7.meter.acceleration, kMaxVoltage
    )

    /** Generation **/

    private fun getConstraints(elevatorUp: Boolean, trajectoryEndpoint: Pose2d) =
        listOf(
            CentripetalAccelerationConstraint(
                if (elevatorUp)
                    kMaxCentripetalAccelerationElevatorUp
                else
                    kMaxCentripetalAccelerationElevatorDown
            ),
            VelocityLimitRadiusConstraint(
                trajectoryEndpoint.translation,
                kVelocityRadiusConstraintRadius,
                kVelocityRadiusConstraintVelocity
            ),
            VelocityLimitRegionConstraint(TrajectoryWaypoints.kHabitatL1Platform, kMaxHabitatVelocity)
        )

    private fun getConstraints(elevatorUp: Boolean, trajectoryEndpoint: TrajectoryWaypoints.Waypoint) =
        getConstraints(elevatorUp, trajectoryEndpoint.position)

    private fun generateTrajectory(
        reversed: Boolean,
        points: List<TrajectoryWaypoints.Waypoint>,
        constraints: List<TimingConstraint<Pose2dWithCurvature>>,
        maxVelocity: LinearVelocity,
        maxAcceleration: LinearAcceleration,
        maxVoltage: Volt,
        optimizeCurvature: Boolean = true
    ): TimedTrajectory<Pose2dWithCurvature> {

        val driveDynamicsConstraint = DifferentialDriveDynamicsConstraint(Constants.DriveConstants.kLowGearDifferentialDrive, maxVoltage)
        val allConstraints = ArrayList<TimingConstraint<Pose2dWithCurvature>>()

        allConstraints.add(driveDynamicsConstraint)
        if (constraints.isNotEmpty()) allConstraints.addAll(constraints)

        return DefaultTrajectoryGenerator.generateTrajectory(
            points.map { it.position },
            allConstraints,
            0.meter.velocity,
            0.meter.velocity,
            maxVelocity,
            maxAcceleration,
            reversed,
            optimizeCurvature
        )
    }

    fun Pose2d.asWaypoint() = TrajectoryWaypoints.Waypoint(this)
}