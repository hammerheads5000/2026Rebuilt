// Copyleft (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class IntakeVisualizer {
    private final LoggedMechanismRoot2d rightRoot;
    private final LoggedMechanismLigament2d rightIntake;

    @AutoLogOutput
    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.5, 0.75);

    private final LoggedMechanismRoot2d leftRoot;
    private final LoggedMechanismLigament2d leftIntake;

    public IntakeVisualizer(String name, Color color) {
        rightRoot = mechanism.getRoot(name + " Right", 0.75 + 0.118, 0.4);
        leftIntake = rightRoot.append(new LoggedMechanismLigament2d(
                "Left Intake", Inches.of(17), Degrees.of(-21), 3, new Color8Bit(color)));
        leftRoot = mechanism.getRoot(name + " Left", 0.75 - 0.118, 0.4);
        rightIntake = leftRoot.append(new LoggedMechanismLigament2d(
                "Right Intake", Inches.of(19.1), Degrees.of(180 + 21), 3, new Color8Bit(color)));
    }

    public void setRightPosition(Distance pos) {
        rightIntake.setLength(pos.plus(Inches.of(19.1)));
        Logger.recordOutput(
                "Intake/Right Pose",
                new Pose3d(
                        0,
                        pos.in(Meters)
                                * Math.cos(Degrees.of(rightIntake.getAngle()).in(Radians)),
                        pos.in(Meters)
                                * Math.sin(Degrees.of(rightIntake.getAngle()).in(Radians)),
                        Rotation3d.kZero));
    }

    public void setLeftPosition(Distance pos) {
        leftIntake.setLength(pos.plus(Inches.of(19.1)));
        Logger.recordOutput(
                "Intake/Left Pose",
                new Pose3d(
                        0,
                        pos.in(Meters)
                                * Math.cos(Degrees.of(leftIntake.getAngle()).in(Radians)),
                        pos.in(Meters)
                                * Math.sin(Degrees.of(leftIntake.getAngle()).in(Radians)),
                        Rotation3d.kZero));
    }
}
