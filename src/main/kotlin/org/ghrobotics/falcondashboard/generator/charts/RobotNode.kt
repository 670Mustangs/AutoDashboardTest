package org.ghrobotics.falcondashboard.generator.charts

import javafx.beans.property.ReadOnlyDoubleProperty
import javafx.beans.property.ReadOnlyProperty
import javafx.beans.property.SimpleDoubleProperty
import javafx.scene.layout.StackPane
import javafx.scene.paint.Color
import org.ghrobotics.falcondashboard.Properties
import org.ghrobotics.falcondashboard.mapprop
import org.ghrobotics.lib.mathematics.units.Rotation2d
import tornadofx.*

open class RobotNode(
    private val robotRotationProperty: ReadOnlyProperty<Rotation2d>,
    scaleProperty: ReadOnlyDoubleProperty
) : StackPane() {

    val robotRotation = SimpleDoubleProperty()

    fun bindRobotRotation() {
        robotRotation
            .bind(mapprop<Rotation2d, Number>(
                robotRotationProperty
            ) { (-value).degree })
    }

    val robotPane = object : StackPane() {
        init {
            style {
                backgroundColor = multi(Color.TRANSPARENT)
                borderColor = multi(box(Color.BLUE))
                borderRadius = multi(box(0.5.em))
                borderWidth = multi(box(0.25.em))
            }
            rotateProperty().bind(robotRotation)
            bindRobotRotation()
            usePrefHeight = true
            usePrefWidth = true
            prefHeightProperty()
                .bind(scaleProperty.multiply(Properties.robotWidth.meter))
            prefWidthProperty()
                .bind(scaleProperty.multiply(Properties.robotLength.meter))
        }
    }

    init {
        children.add(robotPane)
    }

}