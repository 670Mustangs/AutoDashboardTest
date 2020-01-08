package org.ghrobotics.falcondashboard.livevisualizer.charts

import javafx.scene.chart.LineChart
import javafx.scene.chart.NumberAxis
import javafx.scene.chart.XYChart
import javafx.scene.paint.Color
import org.ghrobotics.falcondashboard.Properties
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.SILengthConstants
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.meter
import tornadofx.data
import tornadofx.multi
import tornadofx.style

object FieldChart : LineChart<Number, Number>(
    NumberAxis(0.0, 15.98, 0.5),
    NumberAxis(0.0, 8.21, 0.5)
) {

    private val robotSeries = XYChart.Series<Number, Number>()
    private val pathSeries = XYChart.Series<Number, Number>()
    private val robotBoundingBoxSeries = XYChart.Series<Number, Number>()
    private val visionTargetSeries = XYChart.Series<Number, Number>()

    init {
        style {
            backgroundColor = multi(Color.LIGHTGRAY)
        }
        lookup(".chart-plot-background").style +=
            "-fx-background-image: url(\"chart-background.png\");" +
                "-fx-background-size: stretch;" +
                "-fx-background-position: top right;" +
                "-fx-background-repeat: no-repeat;"

        axisSortingPolicy = LineChart.SortingPolicy.NONE
        isLegendVisible = false
        animated = false
        createSymbols = false

        verticalGridLinesVisible = false
        isHorizontalGridLinesVisible = false

        data.add(robotSeries)
        data.add(pathSeries)
        data.add(robotBoundingBoxSeries)
        data.add(visionTargetSeries)
    }

    override fun resize(width: Double, height: Double) {
        val newWidth = height / 26 * 52
        if (newWidth > width) {
            super.resize(width, width / 52 * 26)
        } else {
            super.resize(newWidth, height)
        }
    }

    fun addRobotPathPose(pose2d: Pose2d) {
        @Suppress("UNCHECKED_CAST")
        robotSeries.data(
            pose2d.translation.x,
            pose2d.translation.y
        )
    }

    fun addPathPose(pose2d: Pose2d) {
        @Suppress("UNCHECKED_CAST")
        pathSeries.data(
            pose2d.translation.x,
            pose2d.translation.y
        )
    }

    fun updateRobotPose(pose2d: Pose2d) {
        robotBoundingBoxSeries.data.clear()
        getRobotBoundingBox(pose2d).forEach {
            robotBoundingBoxSeries.data(
                it.translation.x,
                it.translation.y
            )
        }
    }

    fun updateVisionTargets(newVisionTargets: List<Pose2d>) {
        visionTargetSeries.data.clear()
        newVisionTargets.forEach {
            val data = XYChart.Data<Number, Number>(
                it.translation.x / SILengthConstants.kFeetToMeter,
                it.translation.y / SILengthConstants.kFeetToMeter
            )
            data.node = VisionTargetNode(
                it.rotation,
                (xAxis as NumberAxis).scaleProperty()
            )
            visionTargetSeries.data.add(data)
        }
    }

    private fun getRobotBoundingBox(center: Pose2d): Array<Pose2d> {
        val tl = center.transformBy(
            Pose2d(Translation2d(-Properties.robotLength / 2, Properties.robotWidth / 2))
        )

        val tr = center.transformBy(
            Pose2d(Translation2d(Properties.robotLength / 2, Properties.robotWidth / 2))
        )

        val mid = center.transformBy(Pose2d(Translation2d(Properties.robotLength / 2.0 + 4.inch, 0.inch)))

        val bl = center.transformBy(
            Pose2d(Translation2d(-Properties.robotLength / 2, -Properties.robotWidth / 2))
        )

        val br = center.transformBy(
            Pose2d(Translation2d(Properties.robotLength / 2, -Properties.robotWidth / 2))
        )
        return arrayOf(tl, tr, mid, br, bl, tl)
    }

    fun clear() {
        robotSeries.data.clear()
        pathSeries.data.clear()
    }
}

