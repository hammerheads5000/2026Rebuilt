// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.Intake.IntakeGoal;
import java.util.Map;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intakes extends SubsystemBase {
    public final Intake left;
    public final Intake right;

    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    @AutoLogOutput
    private IntakesGoal goal = IntakesGoal.IDLE;

    @AutoLogOutput
    public Trigger deployLeftTrigger = new Trigger(this::travelingLeft)
            .and(() -> goal == IntakesGoal.AUTOSWITCH)
            .and(DriverStation::isTeleop)
            .debounce(0.08);

    @AutoLogOutput
    public Trigger deployRightTrigger = new Trigger(this::travelingRight)
            .and(() -> goal == IntakesGoal.AUTOSWITCH)
            .and(DriverStation::isTeleop)
            .debounce(0.08);

    private final IntakeVisualizer measuredVisualizer = new IntakeVisualizer("Measured", Color.kGreen);

    /** Creates a new Intake. */
    public Intakes(IntakeIO leftIO, IntakeIO rightIO, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.left = new Intake(leftIO, IntakeSide.Left);
        this.right = new Intake(rightIO, IntakeSide.Right);

        this.chassisSpeedsSupplier = chassisSpeedsSupplier;

        this.deployLeftTrigger.onTrue(deployLeft());
        this.deployRightTrigger.onTrue(deployRight());

        SmartDashboard.putData("Intakes/Deploy Left", left.deploy());
        SmartDashboard.putData("Intakes/Deploy Right", right.deploy());
        SmartDashboard.putData("Intakes/Stow Left", left.stow());
        SmartDashboard.putData("Intakes/Stow Right", right.stow());
        SmartDashboard.putData("Intakes/Autoswitch", setGoal(IntakesGoal.AUTOSWITCH));
        SmartDashboard.putData("Intakes/Stow All", setGoal(IntakesGoal.STOW));
        SmartDashboard.putData("Intakes/Disable", setGoal(IntakesGoal.IDLE));
    }

    public boolean travelingLeft() {
        return chassisSpeedsSupplier.get().vyMetersPerSecond > MIN_SWITCH_ROBOT_VELOCITY.in(MetersPerSecond);
    }

    public boolean travelingRight() {
        return chassisSpeedsSupplier.get().vyMetersPerSecond < -MIN_SWITCH_ROBOT_VELOCITY.in(MetersPerSecond);
    }

    public Command setGoal(IntakesGoal goal) {
        return Commands.runOnce(() -> this.goal = goal)
                .andThen(Commands.select(
                        Map.of(
                                IntakesGoal.AUTOSWITCH,
                                Commands.none(),
                                IntakesGoal.MANUAL,
                                Commands.none(),
                                IntakesGoal.IDLE,
                                left.off().alongWith(right.off()),
                                IntakesGoal.STOW,
                                left.stow().alongWith(right.stow())),
                        () -> goal))
                .withName("Set goal");
    }

    public IntakesGoal getGoal() {
        return goal;
    }

    public Command deployLeft() {
        return Commands.sequence(
                        right.stow()
                                .onlyIf(() -> right.getGoal() != IntakeGoal.DISABLED)
                                .asProxy(),
                        Commands.waitUntil(right.stowedTrigger).onlyIf(() -> right.getGoal() != IntakeGoal.DISABLED),
                        left.deploy().asProxy())
                .withName("Deploy Left");
    }

    public Command deployRight() {
        return Commands.sequence(
                        left.stow()
                                .onlyIf(() -> left.getGoal() != IntakeGoal.DISABLED)
                                .asProxy(),
                        Commands.waitUntil(left.stowedTrigger).onlyIf(() -> left.getGoal() != IntakeGoal.DISABLED),
                        right.deploy().asProxy())
                .withName("Deploy Right");
    }

    public Command switchIntakes() {
        return Commands.either(deployRight(), deployLeft(), this::shouldSwitchToRight);
    }

    private boolean shouldSwitchToRight() {
        boolean leftDisabled = left.getGoal() == IntakeGoal.DISABLED;
        boolean rightDisabled = right.getGoal() == IntakeGoal.DISABLED;
        boolean leftDeploy = left.getGoal() == IntakeGoal.DEPLOYED || left.getGoal() == IntakeGoal.DEPLOYING;
        boolean rightDeploy = right.getGoal() == IntakeGoal.DEPLOYED || right.getGoal() == IntakeGoal.DEPLOYING;
        return (leftDisabled && !rightDeploy)
                || (rightDisabled && leftDeploy)
                || (!(rightDisabled || leftDeploy || rightDeploy) && travelingRight())
                || (!(rightDisabled || leftDisabled) && leftDeploy);
    }

    public Command press() {
        return Commands.either(right.press().asProxy(), left.press().asProxy(), right.deployedTrigger)
                .onlyIf(right.deployedTrigger.or(left.deployedTrigger))
                .withName("Intake Press");
    }

    public Command reverse() {
        return Commands.either(right.reverse().asProxy(), left.reverse().asProxy(), right.deployedTrigger)
                .withName("Reverse intake");
    }

    @Override
    public void periodic() {
        measuredVisualizer.setLeftPosition(left.getPosition());
        measuredVisualizer.setRightPosition(right.getPosition());
        Logger.recordOutput(
                "Intakes/Current Command",
                this.getCurrentCommand() == null
                        ? "None"
                        : this.getCurrentCommand().getName());
    }

    public enum IntakesGoal {
        AUTOSWITCH,
        MANUAL,
        STOW,
        IDLE
    }

    public enum IntakeSide {
        Left,
        Right
    }
}
