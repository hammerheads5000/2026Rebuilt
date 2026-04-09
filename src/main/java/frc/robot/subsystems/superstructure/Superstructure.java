// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import static frc.robot.Constants.TurretConstants.DUCK_TIME;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretGoal;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.Zones;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    private final Turret turret;
    private final Indexer indexer;

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;

    @AutoLogOutput
    private Goal goal = Goal.IDLE;

    @AutoLogOutput
    private Goal nonDuckingGoal = goal;

    @AutoLogOutput
    public final Trigger underTrenchTrigger;

    @AutoLogOutput
    public final Trigger inAllianceZoneTrigger = new Trigger(this::inAllianceZone);

    @AutoLogOutput
    public final Trigger behindTowerTrigger;

    @AutoLogOutput
    private boolean shiftOverride = false;

    private Alert shiftOverrideAlert = new Alert("Shift Override", AlertType.kInfo);

    /**
     * Will trigger when hub is active, or when there is less than ACTIVE_PRESHOOT_TIME until the next active period
     * 100 seconds is an arbitrarily long time so as to not trigger shooting when timeRemainingInCurrentShift gives Optional.empty()
     */
    @AutoLogOutput
    public final Trigger activeHubTrigger =
            new Trigger(() -> HubShiftUtil.getShiftedShiftInfo(shiftOverride).active());

    @AutoLogOutput
    public final Trigger activeInZoneTrigger =
            inAllianceZoneTrigger.and(DriverStation::isTeleop).and(activeHubTrigger);

    @AutoLogOutput
    public final Trigger inactiveInZoneTrigger =
            inAllianceZoneTrigger.and(DriverStation::isTeleop).and(activeHubTrigger.negate());

    @AutoLogOutput
    public final Trigger leaveZoneTrigger = inAllianceZoneTrigger.negate().and(DriverStation::isTeleop);

    private final Map<Goal, Supplier<Command>> goalCommands;

    /** Creates a new Superstructure. */
    public Superstructure(
            Turret turret,
            Indexer indexer,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.turret = turret;
        this.indexer = indexer;
        this.poseSupplier = poseSupplier;
        this.fieldSpeedsSupplier = fieldSpeedsSupplier;

        goalCommands = Map.of(
                Goal.SCORING,
                () -> Commands.sequence(
                                this.turret.setGoal(TurretGoal.SCORING), this.indexer.setGoal(IndexerGoal.ACTIVATING))
                        .withName("Start scoring"),
                Goal.PASSING,
                () -> Commands.sequence(
                                this.turret.setGoal(TurretGoal.PASSING).onlyIf(inAllianceZoneTrigger.negate()),
                                this.indexer.setGoal(IndexerGoal.ACTIVATING).onlyIf(inAllianceZoneTrigger.negate()))
                        .withName("Start passing"),
                Goal.COLLECTING,
                () -> Commands.sequence(this.turret.setGoal(TurretGoal.IDLE), this.indexer.setGoal(IndexerGoal.IDLE))
                        .withName("Start collecting"),
                Goal.DUCKING,
                () -> Commands.sequence(this.turret.setGoal(TurretGoal.DUCKING), this.indexer.setGoal(IndexerGoal.IDLE))
                        .withName("Start ducking"),
                Goal.IDLE,
                () -> Commands.sequence(this.turret.setGoal(TurretGoal.IDLE), this.indexer.setGoal(IndexerGoal.IDLE))
                        .withName("Idle"));

        underTrenchTrigger = Zones.TRENCH_DUCK_ZONES.willContain(poseSupplier, fieldSpeedsSupplier, DUCK_TIME);
        behindTowerTrigger = Zones.TOWER_ZONES.contains(poseSupplier);

        underTrenchTrigger.and(DriverStation::isTeleop).onTrue(this.duck());
        underTrenchTrigger.and(DriverStation::isTeleop).onFalse(this.unduck());
        activeInZoneTrigger.onTrue(this.setGoal(Goal.SCORING));
        inactiveInZoneTrigger.onTrue(this.setGoal(Goal.IDLE));
        leaveZoneTrigger.onTrue(this.setGoal(Goal.COLLECTING).onlyIf(() -> this.goal != Goal.PASSING));
        behindTowerTrigger.and(DriverStation::isTeleop).onTrue(indexer.setGoal(IndexerGoal.IDLE));
        behindTowerTrigger
                .and(DriverStation::isTeleop)
                .onFalse(indexer.setGoal(IndexerGoal.ACTIVATING)
                        .onlyIf(() -> goal == Goal.SCORING || goal == Goal.PASSING));

        // turret.underTrenchTrigger.onTrue(indexer.runOnce(() -> indexer.stop(false)));
        // turret.underTrenchTrigger.onFalse(Commands.defer(() -> indexer.setGoal(indexer.getGoal()), Set.of()));

        SmartDashboard.putData("Overrides/Shift", enableShiftOverride());
    }

    private boolean inAllianceZone() {
        Pose2d pose = poseSupplier.get();
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        return isBlue && pose.getMeasureX().lt(FieldConstants.ALLIANCE_ZONE.plus(Dimensions.FULL_WIDTH.div(2)))
                || !isBlue
                        && pose.getMeasureX()
                                .gt(FieldConstants.FIELD_LENGTH.minus(
                                        FieldConstants.ALLIANCE_ZONE.plus(Dimensions.FULL_WIDTH.div(2))));
    }

    public Command setGoal(Goal newGoal) {
        return Commands.either(
                        this.runOnce(() -> this.nonDuckingGoal = newGoal),
                        // .unless(() -> nonDuckingGoal == Goal.COLLECTING && newGoal == Goal.PASSING),
                        this.runOnce(() -> this.goal = newGoal)
                                .andThen(goalCommands.get(newGoal).get()),
                        underTrenchTrigger.and(() -> newGoal != Goal.DUCKING))
                .withName("Set goal");
    }

    public Command togglePassing() {
        return Commands.either(stopPassCollecting(), this.setGoal(Goal.PASSING), () -> this.goal == Goal.PASSING)
                .withName("Toggle passing");
    }

    /** Handle state logic for transitioning out of PASSING/COLLECTING */
    public Command stopPassCollecting() {
        return Commands.either(this.setGoal(Goal.SCORING), this.setGoal(Goal.COLLECTING), activeInZoneTrigger)
                .onlyIf(() -> this.goal == Goal.PASSING || this.goal == Goal.COLLECTING)
                .withName("Stop passing");
    }

    public Command duck() {
        return this.setGoal(Goal.DUCKING)
                .beforeStarting(() -> nonDuckingGoal = goal)
                .withName("Duck");
    }

    public Command unduck() {
        return Commands.defer(
                        () -> this.setGoal(nonDuckingGoal), // == Goal.SCORING
                        //         ? (inAllianceZone() ? Goal.SCORING : Goal.COLLECTING)
                        //         : nonDuckingGoal),
                        Set.of(this))
                .withName("Unduck");
    }

    public Command enableShiftOverride() {
        return Commands.startEnd(
                        () -> {
                            shiftOverride = true;
                            shiftOverrideAlert.set(true);
                        },
                        () -> {
                            shiftOverride = false;
                            shiftOverrideAlert.set(false);
                        })
                .ignoringDisable(true)
                .withName("Override shift");
    }

    public Goal getGoal() {
        return goal;
    }

    @Override
    public void periodic() {
        Logger.recordOutput(
                "Superstructure/Shift Time", HubShiftUtil.getOfficialShiftInfo().remainingTime());
        Logger.recordOutput(
                "Superstructure/Current Command",
                this.getCurrentCommand() == null
                        ? "None"
                        : this.getCurrentCommand().getName());
    }

    public static enum Goal {
        SCORING,
        PASSING,
        COLLECTING,
        DUCKING,
        IDLE
    }
}
