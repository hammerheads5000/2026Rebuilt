// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class IntakeVisualizer {
    @AutoLogOutput
    private final LoggedMechanism2d leftMechanism = new LoggedMechanism2d(Inches.of(60), Inches.of(30));

    private final LoggedMechanismRoot2d leftRoot;
    private final LoggedMechanismLigament2d leftIntake;

    @AutoLogOutput
    private final LoggedMechanism2d rightMechanism = new LoggedMechanism2d(Inches.of(60), Inches.of(30));

    private final LoggedMechanismRoot2d rightRoot;
    private final LoggedMechanismLigament2d rightIntake;

    public IntakeVisualizer(String name) {
        leftRoot = leftMechanism.getRoot(name + " Left", 0.1, 0.4);
        leftIntake = leftRoot.append(
                new LoggedMechanismLigament2d(name + " Left Intake", Inches.of(20), Degrees.of(180 + 30)));
        rightRoot = rightMechanism.getRoot(name + " Right", -0.1, 0.4);
        rightIntake =
                rightRoot.append(new LoggedMechanismLigament2d(name + " Right Intake", Inches.of(20), Degrees.of(-30)));
    }

    public void setLeftPosition(Distance pos) {
        leftIntake.setLength(pos.plus(Inches.of(20)));
    }

    public void setRightPosition(Distance pos) {
        rightIntake.setLength(pos.plus(Inches.of(20)));
    }
}
