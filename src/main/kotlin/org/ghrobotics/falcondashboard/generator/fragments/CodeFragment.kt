package org.ghrobotics.falcondashboard.generator.fragments

import javafx.scene.layout.Priority
import javafx.scene.text.Font
import kfoenix.jfxtextarea
import org.ghrobotics.falcondashboard.Settings
import org.ghrobotics.falcondashboard.generator.GeneratorView
import org.ghrobotics.lib.mathematics.units.SILengthConstants
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.meter
import tornadofx.*
import java.awt.Desktop
import java.net.URI
import java.text.DecimalFormat

class CodeFragment : Fragment() {
    override val root = vbox {

        title = "Generated Code"

        style {
            padding = box(1.em)
        }

        prefWidth = 800.0
        prefHeight = 500.0

        jfxtextarea {
            font = Font.font("Monospaced")
            isEditable = false

            vgrow = Priority.ALWAYS

            text = buildString {

                val name = Settings.name.value.decapitalize()
                    .replace("\\s+".toRegex(), "")

                val dm = DecimalFormat("##.###")

                append("TrajectoryConfig config = new TrajectoryConfig(\n")
                append("${Settings.maxVelocity.value}")
                append(",")
                append("${Settings.maxAcceleration.value}")
                append(")")
                append(".setKinematics(DriveConstants.kDriveKinematics)")
                append(".addConstraint(autoVoltageConstraint));\n\n")
                append("Trajectory $name = TrajectoryGenerator.generateTrajectory(\n")
                append("   List.of(\n")
                GeneratorView.waypoints.forEach {
                    append(

                        
                        "        new Pose2d(${dm.format(it.translation.x)}, " +
                            "${dm.format(it.translation.y)}, " +
                            "Rotation2d.fromDegree(${dm.format(it.rotation.degree)}))"
                    )
                    if (it != GeneratorView.waypoints.last()) append(",")
                    append("\n")
                }
                append("    ),\n")
                append("    config);\n")
            }
        }
        vbox {
            style {
                padding = box(0.5.em, 0.em, 0.em, 0.em)
            }
        }
    }
}