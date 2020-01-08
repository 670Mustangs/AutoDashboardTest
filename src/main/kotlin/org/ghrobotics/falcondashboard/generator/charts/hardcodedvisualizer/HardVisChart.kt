package org.ghrobotics.falcondashboard.generator.charts.hardcodedvisualizer

import javafx.beans.property.SimpleObjectProperty
import javafx.scene.chart.LineChart
import javafx.scene.chart.NumberAxis
import javafx.scene.chart.XYChart
import javafx.scene.control.Tooltip
import javafx.scene.input.MouseButton
import javafx.scene.paint.Color
import javafx.scene.paint.Paint
import org.ghrobotics.falcondashboard.generator.GeneratorView
import org.ghrobotics.falcondashboard.generator.charts.PositionChart
import org.ghrobotics.falcondashboard.generator.charts.PositionNode
import org.ghrobotics.falcondashboard.generator.charts.hardcodedvisualizer.paths.TrajectoryFactory
import org.ghrobotics.falcondashboard.generator.charts.hardcodedvisualizer.paths.TrajectoryFactory.asWaypoint
import org.ghrobotics.falcondashboard.generator.charts.hardcodedvisualizer.paths.TrajectoryWaypoints
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.twodim.trajectory.DefaultTrajectoryGenerator
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TrajectorySamplePoint
import org.ghrobotics.lib.mathematics.units.*
import org.ghrobotics.lib.mathematics.units.feet
import tornadofx.*

object HardVisChart : LineChart<Number, Number>(
    NumberAxis(0.0, 15.98, 1.0),
    NumberAxis(0.0, 8.21, 1.0)
) {

    private val seriesXY = XYChart.Series<Number, Number>()
    private val seriesWayPoints = XYChart.Series<Number, Number>()

    val trajList = listOf(TrajectoryFactory.sideStartReversedToRocketFPrepare, TrajectoryFactory.rocketFPrepareToRocketF)
    var trajIterator = trajList.listIterator()

    val waypoints = observableList(
        TrajectoryWaypoints.kSideStartReversed.asWaypoint().position,
        Pose2d(15.214.feet, 8.7.feet, 165.degree).asWaypoint().position,
        Pose2d(22.488.feet, 5.639.feet, 143.degree).asWaypoint().position,
        Pose2d(24.074.feet, 3.753.feet, (-143).degree).asWaypoint().position,
//        Pose2d(24.074.feet, 3.753.feet, -143.degree).asWaypoint().position,
        TrajectoryFactory.rocketFAdjusted.position.transformBy(Pose2d(Translation2d((-4).inch, 0.inch))).asWaypoint().position
    )

    private val trajectory = SimpleObjectProperty(DefaultTrajectoryGenerator.baseline)

    init {
        style {
            backgroundColor = MultiValue(arrayOf<Paint>(Color.LIGHTGRAY))
        }
        lookup(".chart-plot-background").style +=
            "-fx-background-image: url(\"chart-background.png\");" +
                    "-fx-background-size: stretch;" +
                    "-fx-background-position: top right;" +
                    "-fx-background-repeat: no-repeat;"

        axisSortingPolicy = LineChart.SortingPolicy.NONE
        isLegendVisible = false
        animated = false
        createSymbols = true
        verticalGridLinesVisible = false
        isHorizontalGridLinesVisible = false

        data.add(seriesXY)
        data.add(seriesWayPoints)

        setOnMouseClicked {
            if (it.button == MouseButton.PRIMARY) {
                if (it.clickCount == 2) {
                    if(!trajIterator.hasNext()) trajIterator = trajList.listIterator()

                    trajectory.set(trajIterator.next())

                }
            }
        }

        seriesWayPoints.data
            .bind(waypoints) {
                val data = XYChart.Data<Number, Number>(
                    it.translation.x,
                    it.translation.y
                )
                val currentPose2d = SimpleObjectProperty(it)
                currentPose2d.addListener { _, oldPose, newPose ->
                    waypoints[waypoints.indexOf(oldPose)] = newPose
                }
                val node = PositionNode(
                    data,
                    (xAxis as NumberAxis),
                    (yAxis as NumberAxis),
                    currentPose2d
                )
                data.node = node

                data
            }

        updateSeriesXY()
        trajectory.addListener { _, _, _ ->
            updateSeriesXY()
        }
    }

    private fun updateSeriesXY() {
        seriesXY.data.clear()

        val iterator =  trajectory.value.iterator()

        while (!iterator.isDone) {
            val point: TrajectorySamplePoint<TimedEntry<Pose2dWithCurvature>> =
                iterator.advance(0.02.second)
            val data = seriesXY.data(
                point.state.state.pose.translation.x,
                point.state.state.pose.translation.y,
                point.state.state.pose.rotation.degree
            )
            Tooltip.install(
                data.node,
                Tooltip(
                    "%2.2f meter, %2.2f meter, %2.2f degrees".format(
                        data.xValue,
                        data.yValue,
                        data.extraValue
                    )
                )
            )
            data.node.toBack()
        }
    }

    override fun resize(width: Double, height: Double) = super.resize(height / 26 * 52, height)

}