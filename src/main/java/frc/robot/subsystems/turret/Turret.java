// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.TurretConstants.*;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualPD;
import frc.robot.util.Zones;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;

    @AutoLogOutput
    Translation3d currentTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? FieldConstants.HUB_BLUE
            : FieldConstants.HUB_RED;

    @AutoLogOutput
    private TurretGoal goal = TurretGoal.OFF;

    private final TurretVisualizer turretVisualizer;

    private final Trigger hoodStalledTrigger;

    private final LoggedTunableNumber turnKP = new LoggedTunableNumber("Turret/Turn/kP", TURN_GAINS.kP);
    private final LoggedTunableNumber turnKD = new LoggedTunableNumber("Turret/Turn/kD", TURN_GAINS.kD);
    private final LoggedTunableNumber turnKV = new LoggedTunableNumber("Turret/Turn/kV", TURN_GAINS.kV);
    private final LoggedTunableNumber turnKS = new LoggedTunableNumber("Turret/Turn/kS", TURN_GAINS.kS);
    private final LoggedTunableNumber hoodKP = new LoggedTunableNumber("Turret/Hood/kP", HOOD_GAINS.kP);
    private final LoggedTunableNumber hoodKD = new LoggedTunableNumber("Turret/Hood/kD", HOOD_GAINS.kD);
    private final LoggedTunableNumber hoodKS = new LoggedTunableNumber("Turret/Hood/kS", HOOD_GAINS.kS);
    private final LoggedTunableNumber flywheelKP = new LoggedTunableNumber("Turret/Flywheel/kP", FLYWHEEL_GAINS.kP);
    private final LoggedTunableNumber flywheelKD = new LoggedTunableNumber("Turret/Flywheel/kD", FLYWHEEL_GAINS.kD);
    private final LoggedTunableNumber flywheelKV = new LoggedTunableNumber("Turret/Flywheel/kV", FLYWHEEL_GAINS.kV);
    private final LoggedTunableNumber flywheelKS = new LoggedTunableNumber("Turret/Flywheel/kS", FLYWHEEL_GAINS.kS);

    private final LoggedTunableNumber tuningFlywheelSpeed = new LoggedTunableNumber("Turret/Tuning/FlywheelRPM", 0);
    private final LoggedTunableNumber tuningHoodAngle =
            new LoggedTunableNumber("Turret/Tuning/HoodAngleDegrees", MIN_HOOD_ANGLE.in(Degrees));

    public final Trigger turnaroundZoneMaxTrigger = new Trigger(this::inTurnaroundZoneMax).debounce(0.05);
    public final Trigger turnaroundZoneMinTrigger = new Trigger(this::inTurnaroundZoneMin).debounce(0.05);

    @AutoLogOutput
    public final Trigger underTrenchTrigger;

    @AutoLogOutput
    private TurretGoal nonDuckingGoal = goal;

    @AutoLogOutput
    private MutAngularVelocity flywheelFudgeFactor = RPM.of(0).mutableCopy();

    private Angle[] turnVelFilterPos =
            new Angle[] {Radians.zero(), Radians.zero(), Radians.zero(), Radians.zero(), Radians.zero()};
    private Time[] turnVelFilterTimes =
            new Time[] {Seconds.zero(), Seconds.zero(), Seconds.zero(), Seconds.zero(), Seconds.zero()};

    @AutoLogOutput
    private AngularVelocity velocitySetpoint = RadiansPerSecond.zero();

    private final Alert flywheelDisconnectedAlert = new Alert("Turret Flywheel Motor Disconnected!", AlertType.kError);
    private final Alert hoodDisconnectedAlert = new Alert("Turret Hood Motor Disconnected!", AlertType.kError);
    private final Alert turnDisconnectedAlert = new Alert("Turret Turn Motor Disconnected!", AlertType.kError);

    public Turret(TurretIO io, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.io = io;
        this.inputs = new TurretIOInputsAutoLogged();
        this.poseSupplier = poseSupplier;
        this.fieldSpeedsSupplier = fieldSpeedsSupplier;

        io.zeroHoodPosition();
        setTarget(FieldConstants.HUB_BLUE);

        VirtualPD.registerMotor(() -> inputs.hoodSupplyCurrent, "Turret");
        VirtualPD.registerMotor(() -> inputs.turnSupplyCurrent, "Turret");
        VirtualPD.registerMotor(() -> inputs.flywheelLeaderSupplyCurrent, "Turret");
        VirtualPD.registerMotor(() -> inputs.flywheelFollowerSupplyCurrent, "Turret");

        hoodStalledTrigger = new Trigger(() -> inputs.hoodCurrent.abs(Amps) >= HOOD_STALL_CURRENT.abs(Amps)
                && inputs.hoodVelocity.abs(RadiansPerSecond) <= HOOD_STALL_ANGULAR_VELOCITY.abs(RadiansPerSecond));

        underTrenchTrigger = Zones.TRENCH_DUCK_ZONES.willContain(poseSupplier, fieldSpeedsSupplier, DUCK_TIME);

        underTrenchTrigger.onTrue(duck());
        underTrenchTrigger.onFalse(unduck());

        turretVisualizer = new TurretVisualizer(
                () -> new Pose3d(poseSupplier
                                .get()
                                .rotateAround(poseSupplier.get().getTranslation(), new Rotation2d(inputs.turnPosition)))
                        .transformBy(ROBOT_TO_TURRET_TRANSFORM),
                fieldSpeedsSupplier);

        SmartDashboard.putData("Overrides/Turret Disable", disable());
        SmartDashboard.putData("Overrides/Turret Manual", manualOverride());
        SmartDashboard.putData("Turret/Fudge Up", increaseFudgeFactor());
        SmartDashboard.putData("Turret/Fudge Down", decreaseFudgeFactor());
    }

    public Command setGoal(TurretGoal goal) {
        return this.runOnce(() -> {
                    if (this.goal == TurretGoal.DISABLED || this.goal == TurretGoal.MANUAL_OVERRIDE) {
                        return;
                    }
                    // don't interrupt ducking with another goal
                    if (goal != TurretGoal.DISABLED
                            && this.goal == TurretGoal.DUCKING
                            && underTrenchTrigger.getAsBoolean()) {
                        this.nonDuckingGoal = goal;
                        return;
                    }
                    this.goal = goal;
                    switch (goal) {
                        case SCORING:
                            setTarget(FieldConstants.HUB_BLUE);
                            break;
                        case PASSING:
                            setTarget(getPassingTarget(poseSupplier.get()));
                            break;
                        case IDLE:
                            io.stopFlywheel();
                            io.stopHood();
                            io.stopTurn();
                            break;
                        case TUNING:
                            setTarget(FieldConstants.HUB_BLUE);
                            break;
                        case DUCKING:
                            io.setHoodAngle(MIN_HOOD_ANGLE);
                            Angle desired =
                                    fieldSpeedsSupplier.get().vxMetersPerSecond > 0 ? Degrees.zero() : Degrees.of(180);
                            io.setTurnSetpoint(
                                    TurretCalculator.calculateAzimuthAngle(
                                            poseSupplier.get(), desired, inputs.turnPosition),
                                    RadiansPerSecond.of(0));
                            break;
                        case OFF:
                            io.stopFlywheel();
                            io.stopHood();
                            io.stopTurn();
                            break;
                        case DISABLED:
                            io.stopFlywheel();
                            io.stopHood();
                            io.stopTurn();
                            break;
                        case MANUAL_OVERRIDE:
                            io.stopFlywheel();
                            io.setHoodAngle(MIN_HOOD_ANGLE);
                            io.setTurnSetpoint(Radians.of(0), RadiansPerSecond.of(0));
                            break;
                    }
                })
                .withName("Set Turret Goal");
    }

    private Command increaseFudgeFactor() {
        return Commands.runOnce(() -> this.flywheelFudgeFactor.mut_plus(FLYWHEEL_FUDGE_AMOUNT))
                .withName("Fudge Up");
    }

    private Command decreaseFudgeFactor() {
        return Commands.runOnce(() -> this.flywheelFudgeFactor.mut_minus(FLYWHEEL_FUDGE_AMOUNT))
                .withName("Fudge Down");
    }

    private Command duck() {
        return this.setGoal(TurretGoal.DUCKING).beforeStarting(() -> nonDuckingGoal = this.goal);
    }

    private Command unduck() {
        return Commands.defer(() -> this.setGoal(nonDuckingGoal), Set.of());
    }

    public TurretGoal getGoal() {
        return goal;
    }

    public Command setTurnPosition(Angle position) {
        return this.runOnce(() -> io.setTurnSetpoint(position, RadiansPerSecond.zero()));
    }

    public Command setHoodPosition(Angle angle) {
        return this.runOnce(() -> io.setHoodAngle(angle));
    }

    public Command setFlywheelSpeed(AngularVelocity speed) {
        return this.runOnce(() -> io.setFlywheelSpeed(speed));
    }

    public void setTarget(Translation3d target) {
        currentTarget = target;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            Translation2d flipped = FlippingUtil.flipFieldPosition(target.toTranslation2d());
            currentTarget = new Translation3d(flipped.getX(), flipped.getY(), target.getZ());
        }
    }

    private Translation3d getPassingTarget(Pose2d pose) {
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        boolean onBlueLeftSide = poseSupplier.get().getMeasureY().gt(FieldConstants.FIELD_WIDTH.div(2));

        return isBlue == onBlueLeftSide ? PASSING_SPOT_LEFT : PASSING_SPOT_RIGHT;
    }

    private boolean inTurnaroundZoneMax() {
        return inputs.turnPosition.isNear(MAX_TURN_ANGLE, TURNAROUND_ZONE);
    }

    private boolean inTurnaroundZoneMin() {
        return inputs.turnPosition.isNear(MIN_TURN_ANGLE, TURNAROUND_ZONE);
    }

    private void updateVelocitySetpoint() {
        MutAngularVelocity sumVelocities = RadiansPerSecond.zero().mutableCopy();

        // update filter window
        for (int i = turnVelFilterPos.length - 1; i >= 1; i--) {
            Angle deltaAngle = turnVelFilterPos[i - 1].minus(turnVelFilterPos[i]);
            Time deltaTime = turnVelFilterTimes[i - 1].minus(turnVelFilterTimes[i]);
            sumVelocities.mut_plus(deltaAngle.div(deltaTime));

            turnVelFilterPos[i] = turnVelFilterPos[i - 1];
            turnVelFilterTimes[i] = turnVelFilterTimes[i - 1];
        }
        turnVelFilterPos[0] = inputs.turnSetpoint;
        turnVelFilterTimes[0] = Microseconds.of(RobotController.getFPGATime());

        velocitySetpoint = sumVelocities.copy();
    }

    public Command disable() {
        return setGoal(TurretGoal.DISABLED)
                .andThen(Commands.idle())
                .finallyDo(() -> goal = TurretGoal.OFF)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("Disable Turret");
    }

    public Command manualOverride() {
        return setGoal(TurretGoal.MANUAL_OVERRIDE)
                .andThen(Commands.idle())
                .finallyDo(() -> {
                    if (goal != TurretGoal.DISABLED) {
                        goal = TurretGoal.OFF;
                    }
                })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("Turret Manual Override");
    }

    public Command manualScore() {
        return Commands.startEnd(
                        () -> {
                            io.setFlywheelSpeed(FLYWHEEL_SCORING_OVERRIDE.plus(flywheelFudgeFactor));
                            io.setHoodAngle(HOOD_SCORING_OVERRIDE);
                            io.setTurnSetpoint(Radians.zero(), RadiansPerSecond.zero());
                        },
                        () -> {
                            io.stopFlywheel();
                            io.setHoodAngle(Radians.zero());
                            io.setTurnSetpoint(Radians.zero(), RadiansPerSecond.zero());
                        })
                .onlyIf(() -> goal == TurretGoal.MANUAL_OVERRIDE)
                .withName("Turret Manual Score");
    }

    public Command manualPass() {
        return Commands.startEnd(
                        () -> {
                            io.setFlywheelSpeed(FLYWHEEL_PASSING_OVERRIDE.plus(flywheelFudgeFactor));
                            io.setHoodAngle(HOOD_PASSING_OVERRIDE);
                            io.setTurnSetpoint(Radians.zero(), RadiansPerSecond.zero());
                        },
                        () -> {
                            io.stopFlywheel();
                            io.setHoodAngle(Radians.zero());
                            io.setTurnSetpoint(Radians.zero(), RadiansPerSecond.zero());
                        })
                .onlyIf(() -> goal == TurretGoal.MANUAL_OVERRIDE)
                .withName("Turret Manual Pass");
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);

        Pose2d pose = poseSupplier.get();

        switch (goal) {
            case SCORING:
                calculateShot(pose);
                break;
            case PASSING:
                calculateShot(pose);
                setTarget(getPassingTarget(pose));
                break;
            case TUNING:
                io.setFlywheelSpeed(RPM.of(tuningFlywheelSpeed.get()));
                io.setHoodAngle(Degrees.of(tuningHoodAngle.get()));
                io.setTurnSetpoint(
                        TurretCalculator.calculateAzimuthAngle(pose, currentTarget, inputs.turnPosition), RPM.zero());
                break;
            default:
                break;
        }

        Logger.recordOutput("Turret/Distance To Target", TurretCalculator.getDistanceToTarget(pose, currentTarget));

        turretVisualizer.update3dPose(inputs.turnPosition, inputs.hoodPosition);
        updateTunables();

        flywheelDisconnectedAlert.set(!inputs.flywheelMotorConnected && Constants.CURRENT_MODE != Mode.SIM);
        hoodDisconnectedAlert.set(!inputs.hoodMotorConnected && Constants.CURRENT_MODE != Mode.SIM);
        turnDisconnectedAlert.set(!inputs.turnMotorConnected && Constants.CURRENT_MODE != Mode.SIM);
    }

    private void calculateShot(Pose2d robotPose) {
        ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

        ShotData calculatedShot;
        if (Constants.CURRENT_MODE == Mode.REAL) {
            calculatedShot = TurretCalculator.iterativeMovingShotFromMap(
                    robotPose, fieldSpeeds, currentTarget, LOOKAHEAD_ITERATIONS);
        } else {
            calculatedShot = TurretCalculator.iterativeMovingShotFromFunnelClearance(
                    robotPose, fieldSpeeds, currentTarget, LOOKAHEAD_ITERATIONS);
        }
        Angle azimuthAngle =
                TurretCalculator.calculateAzimuthAngle(robotPose, calculatedShot.target(), inputs.turnPosition);
        // AngularVelocity azimuthVelocity = RadiansPerSecond.of(-fieldSpeeds.omegaRadiansPerSecond);

        updateVelocitySetpoint();

        io.setTurnSetpoint(azimuthAngle, velocitySetpoint);
        io.setHoodAngle(calculatedShot.getHoodAngle());
        io.setFlywheelSpeed(calculatedShot.getAngularExitVelocity().plus(flywheelFudgeFactor));

        Logger.recordOutput("Turret/Shot", calculatedShot);
    }

    public Command zeroHoodSequence() {
        return Commands.sequence(
                this.runOnce(() -> io.setHoodOut(HOOD_ZEROING_VOLTAGE)),
                Commands.waitSeconds(0.1),
                Commands.waitUntil(hoodStalledTrigger::getAsBoolean),
                this.runOnce(io::stopHood),
                Commands.waitSeconds(0.2),
                this.runOnce(() -> {
                    io.zeroHoodPosition();
                    io.setHoodAngle(MIN_HOOD_ANGLE);
                }));
    }

    private void updateTunables() {
        if (turnKP.hasChanged(hashCode())
                || turnKD.hasChanged(hashCode())
                || turnKV.hasChanged(hashCode())
                || turnKS.hasChanged(hashCode())) {
            io.setTurnPID(turnKP.get(), turnKD.get(), turnKV.get(), turnKS.get());
        }

        if (hoodKP.hasChanged(hashCode()) || hoodKD.hasChanged(hashCode()) || hoodKS.hasChanged(hashCode())) {
            io.setHoodPID(hoodKP.get(), hoodKD.get(), hoodKS.get());
        }

        if (flywheelKP.hasChanged(hashCode())
                || flywheelKD.hasChanged(hashCode())
                || flywheelKV.hasChanged(hashCode())
                || flywheelKS.hasChanged(hashCode())) {
            io.setFlywheelPID(flywheelKP.get(), flywheelKD.get(), flywheelKV.get(), flywheelKS.get());
        }
    }

    @Override
    public void simulationPeriodic() {
        turretVisualizer.updateFuel(
                TurretCalculator.angularToLinearVelocity(inputs.flywheelSpeed, FLYWHEEL_RADIUS), inputs.hoodPosition);
    }

    public enum TurretGoal {
        SCORING,
        PASSING,
        IDLE,
        TUNING,
        DUCKING,
        OFF,
        MANUAL_OVERRIDE,
        DISABLED
    }
}
