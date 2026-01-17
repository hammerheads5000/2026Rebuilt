// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class IntakeVisualizer {
    private final LoggedMechanismRoot2d leftRoot;
    private final LoggedMechanismLigament2d leftIntake;

    @AutoLogOutput
    private final LoggedMechanism2d mechanism = new LoggedMechanism2d(1.5, 0.75);

    private final LoggedMechanismRoot2d rightRoot;
    private final LoggedMechanismLigament2d rightIntake;

    public IntakeVisualizer(String name, Color color) {
        leftRoot = mechanism.getRoot(name + " Left", 0.85, 0.4);
        leftIntake = leftRoot.append(new LoggedMechanismLigament2d(
                "Left Intake", Inches.of(17), Degrees.of(180 + 20), 3, new Color8Bit(color)));
        rightRoot = mechanism.getRoot(name + " Right", 0.65, 0.4);
        rightIntake = rightRoot.append(
                new LoggedMechanismLigament2d("Right Intake", Inches.of(17), Degrees.of(-20), 3, new Color8Bit(color)));
    }

    public void setLeftPosition(Distance pos) {
        leftIntake.setLength(pos.plus(Inches.of(17)));
    }

    public void setRightPosition(Distance pos) {
        rightIntake.setLength(pos.plus(Inches.of(17)));
    }
}
