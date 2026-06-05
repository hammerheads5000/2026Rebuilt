// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.TurretConstants.*;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
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
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;
    private final Supplier<Angle> pitchSupplier;
    private final Supplier<Angle> rollSupplier;

    @AutoLogOutput
    public Translation3d currentTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? FieldConstants.HUB_BLUE
            : FieldConstants.HUB_RED;

    @AutoLogOutput
    private TurretGoal goal = TurretGoal.IDLE;

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

    public final LoggedDashboardChooser<Translation3d> evilTargetChooser =
            new LoggedDashboardChooser<Translation3d>("Who to be an asshole to");

    @AutoLogOutput
    private double flywheelFudgeFactor = 1;

    @AutoLogOutput
    private double turnTrimDeg = 0;

    @AutoLogOutput
    private Time tofFudgeFactor = Seconds.zero();

    @AutoLogOutput
    public final Trigger atTurnSetpoint;

    private LinearVelocity prevTangentialVel = MetersPerSecond.zero();
    private long prevTimeMicros = 0;

    private final Alert flywheelDisconnectedAlert = new Alert("Turret Flywheel Motor Disconnected!", AlertType.kError);
    private final Alert hoodDisconnectedAlert = new Alert("Turret Hood Motor Disconnected!", AlertType.kError);
    private final Alert turnDisconnectedAlert = new Alert("Turret Turn Motor Disconnected!", AlertType.kError);
    private final Alert disabledAlert = new Alert("Turret Disabled", AlertType.kWarning);
    private final Alert manualOverrideAlert = new Alert("Turret Override", AlertType.kWarning);

    public Turret(
            TurretIO io,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier,
            Supplier<ChassisSpeeds> requestedFieldSpeedsSupplier,
            Supplier<Angle> pitchSupplier,
            Supplier<Angle> rollSupplier) {
        this.io = io;
        this.inputs = new TurretIOInputsAutoLogged();
        this.poseSupplier = poseSupplier;
        this.fieldSpeedsSupplier = fieldSpeedsSupplier;
        this.pitchSupplier = pitchSupplier;
        this.rollSupplier = rollSupplier;
        io.zeroHoodPosition();
        setTarget(FieldConstants.HUB_BLUE);

        VirtualPD.registerMotor(() -> inputs.hoodSupplyCurrent, "Turret");
        VirtualPD.registerMotor(() -> inputs.turnSupplyCurrent, "Turret");
        VirtualPD.registerMotor(() -> inputs.flywheelLeaderSupplyCurrent, "Turret");
        VirtualPD.registerMotor(() -> inputs.flywheelFollowerSupplyCurrent, "Turret");

        hoodStalledTrigger = new Trigger(() -> inputs.hoodCurrent.abs(Amps) >= HOOD_STALL_CURRENT.abs(Amps)
                && inputs.hoodVelocity.abs(RadiansPerSecond) <= HOOD_STALL_ANGULAR_VELOCITY.abs(RadiansPerSecond));

        turretVisualizer = new TurretVisualizer(
                () -> new Pose3d(poseSupplier
                                .get()
                                .rotateAround(poseSupplier.get().getTranslation(), new Rotation2d(inputs.turnPosition)))
                        .transformBy(ROBOT_TO_TURRET_TRANSFORM),
                fieldSpeedsSupplier);

        atTurnSetpoint =
                new Trigger(() -> inputs.turnPosition.isNear(inputs.turnSetpoint, TURN_TOLERANCE)).debounce(0.1);

        SmartDashboard.putData("Overrides/Turret Disable", disable());
        SmartDashboard.putData("Overrides/Turret Manual", manualOverride());
        SmartDashboard.putData("Turret/Fudge Up", increaseFudgeFactor());
        SmartDashboard.putData("Turret/Fudge Down", decreaseFudgeFactor());
        SmartDashboard.putData("Turret/Turn Trim Left", increaseTurnTrim());
        SmartDashboard.putData("Turret/Turn Trim Right", decreaseTurnTrim());
        SmartDashboard.putData("Turret/Reset Encoder", resetTurnEncoder());
        SmartDashboard.putData("Turret/ToF Fudge Up", increaseToFFudgeFactor());
        SmartDashboard.putData("Turret/ToF Fudge Down", decreaseToFFudgeFactor());

        evilTargetChooser.addOption("Right", FieldConstants.DRIVER_STATIONS[0]);
        evilTargetChooser.addDefaultOption("Center", FieldConstants.DRIVER_STATIONS[1]);
        evilTargetChooser.addOption("Left", FieldConstants.DRIVER_STATIONS[2]);

        Logger.recordOutput("Turret/Shot", new ShotData(0, 0)); // log struct at start to avoid loop overruns
    }

    public Command setGoal(TurretGoal goal) {
        return Commands.runOnce(() -> {
                    if (this.goal == TurretGoal.DISABLED || this.goal == TurretGoal.MANUAL_OVERRIDE) {
                        return;
                    }
                    this.goal = goal;
                    switch (goal) {
                        case SCORING:
                            setTarget(FieldConstants.HUB_BLUE);
                            break;
                        case PASSING:
                            setTarget(getPassingTarget(poseSupplier.get()), false);
                            break;
                        case EVIL:
                            setTarget(evilTargetChooser.get());
                            break;
                        case IDLE:
                            io.stopFlywheel();
                            io.setHoodAngle(MIN_HOOD_ANGLE);
                            io.stopTurn();
                            break;
                        case TUNING:
                            // setTarget(getPassingTarget(poseSupplier.get()));
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
                        case DISABLED:
                            io.stopFlywheel();
                            io.stopHood();
                            io.stopTurn();
                            break;
                        case MANUAL_OVERRIDE:
                            io.stopFlywheel();
                            io.setHoodAngle(MIN_HOOD_ANGLE);
                            io.setTurnSetpoint(Degrees.of(turnTrimDeg), RadiansPerSecond.of(0));
                            break;
                    }
                })
                .withName("Set Turret Goal");
    }

    public Command increaseTurnTrim() {
        return Commands.runOnce(() -> turnTrimDeg += TURN_TRIM_AMOUNT.in(Degrees))
                .ignoringDisable(true)
                .withName("Trim Left");
    }

    public Command decreaseTurnTrim() {
        return Commands.runOnce(() -> turnTrimDeg -= TURN_TRIM_AMOUNT.in(Degrees))
                .ignoringDisable(true)
                .withName("Trim Right");
    }

    public Command increaseFudgeFactor() {
        return Commands.runOnce(() -> this.flywheelFudgeFactor =
                        Math.round(100 * flywheelFudgeFactor * (1 + FLYWHEEL_FUDGE_AMOUNT)) / 100.0)
                .ignoringDisable(true)
                .withName("Fudge Up");
    }

    public Command decreaseFudgeFactor() {
        return Commands.runOnce(() -> this.flywheelFudgeFactor =
                        Math.round(100 * flywheelFudgeFactor * (1 - FLYWHEEL_FUDGE_AMOUNT)) / 100.0)
                .ignoringDisable(true)
                .withName("Fudge Down");
    }

    public Command increaseToFFudgeFactor() {
        return Commands.runOnce(() -> this.tofFudgeFactor = tofFudgeFactor.plus(Seconds.of(0.01)))
                .ignoringDisable(true)
                .withName("ToF Fudge Up");
    }

    public Command decreaseToFFudgeFactor() {
        return Commands.runOnce(() -> this.tofFudgeFactor = tofFudgeFactor.minus(Seconds.of(0.01)))
                .ignoringDisable(true)
                .withName("ToF Fudge Down");
    }

    public TurretGoal getGoal() {
        return goal;
    }

    public Command setTurnPosition(Angle position) {
        return Commands.runOnce(
                () -> io.setTurnSetpoint(position.plus(Degrees.of(turnTrimDeg)), RadiansPerSecond.zero()));
    }

    public Command setHoodPosition(Angle angle) {
        return Commands.runOnce(() -> io.setHoodAngle(angle));
    }

    public Command setFlywheelSpeed(AngularVelocity speed) {
        return Commands.runOnce(() -> io.setFlywheelSpeed(speed));
    }

    public void setTarget(Translation3d target) {
        setTarget(target, DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);
    }

    public void setTarget(Translation3d target, boolean flip) {
        if (flip) {
            Translation2d flipped = FlippingUtil.flipFieldPosition(target.toTranslation2d());
            currentTarget = new Translation3d(flipped.getX(), flipped.getY(), target.getZ());
        } else {
            currentTarget = target;
        }
    }

    private Translation3d getPassingTarget(Pose2d pose) {
        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        boolean onBlueLeftSide = poseSupplier.get().getMeasureY().gt(FieldConstants.FIELD_WIDTH.div(2));

        Translation3d target = isBlue == onBlueLeftSide ? PASSING_SPOT_LEFT : PASSING_SPOT_RIGHT;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            target = new Translation3d(FlippingUtil.flipFieldPosition(target.toTranslation2d()));
        }

        Translation2d dir = pose.getTranslation().minus(target.toTranslation2d());
        Distance rollDistance = getRollDistance(dir.getNorm());
        dir = dir.div(dir.getNorm()); // 1 m
        dir = dir.times(rollDistance.in(Meters));

        target = target.plus(new Translation3d(dir));
        return target;
    }

    private static Distance getRollDistance(double distance) {
        double rollingDist = distance * ROLL_PROPORTION;
        return Meters.of(MathUtil.clamp(rollingDist, 0, MAX_PASSING_ROLL_DISTANCE.in(Meters)));
    }

    private boolean inTurnaroundZoneMax() {
        return inputs.turnPosition.isNear(MAX_TURN_ANGLE, TURNAROUND_ZONE);
    }

    private boolean inTurnaroundZoneMin() {
        return inputs.turnPosition.isNear(MIN_TURN_ANGLE, TURNAROUND_ZONE);
    }

    private AngularVelocity getVelocitySetpoint(
            Translation2d robotToTarget, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
        LinearVelocity tangentialVel = getTangentialVelocity(robotToTarget, fieldSpeeds);
        return RadiansPerSecond.of(-fieldSpeeds.omegaRadiansPerSecond)
                .plus(tangentVelToAngularVel(robotToTarget, tangentialVel))
                .plus(angularVelSOTM(robotToTarget, fieldSpeeds, timeOfFlight));
    }

    private static LinearVelocity getTangentialComponent(Translation2d robotToTarget, Translation2d vec) {
        Translation2d dir = robotToTarget.div(robotToTarget.getNorm());
        dir = dir.rotateBy(Rotation2d.kCW_90deg);
        return MetersPerSecond.of(vec.dot(dir));
    }

    private static LinearVelocity getTangentialVelocity(Translation2d robotToTarget, ChassisSpeeds fieldSpeeds) {
        return getTangentialComponent(
                robotToTarget, new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond));
    }

    private LinearAcceleration getTangentialAcceleration(Translation2d robotToTarget, ChassisSpeeds fieldSpeeds) {
        LinearVelocity tangentialVel = getTangentialVelocity(robotToTarget, fieldSpeeds);
        LinearAcceleration tangentialAccel = (tangentialVel.minus(prevTangentialVel))
                .div(Microseconds.of(RobotController.getFPGATime() - prevTimeMicros));
        prevTangentialVel = tangentialVel;
        prevTimeMicros = RobotController.getFPGATime();
        return tangentialAccel;
    }

    private static AngularVelocity tangentVelToAngularVel(Translation2d robotToTarget, LinearVelocity tangentialVel) {
        double radius = robotToTarget.getNorm();
        if (radius == 0) {
            return RadiansPerSecond.zero();
        }
        return RadiansPerSecond.of(tangentialVel.in(MetersPerSecond) / radius);
    }

    private AngularVelocity angularVelSOTM(Translation2d robotToTarget, ChassisSpeeds fieldSpeeds, Time timeOfFlight) {
        double tangentialVel = getTangentialVelocity(robotToTarget, fieldSpeeds).in(MetersPerSecond);
        double tangentialAccel =
                getTangentialAcceleration(robotToTarget, fieldSpeeds).in(MetersPerSecondPerSecond);
        double distance = robotToTarget.getNorm();
        double tof = timeOfFlight.in(Seconds);
        if (distance == 0) {
            return RadiansPerSecond.zero();
        }
        return RadiansPerSecond.of(
                tof * tangentialAccel * distance / (Math.pow(distance, 2) + Math.pow(tangentialVel * tof, 2)));
    }

    public Command disable() {
        return setGoal(TurretGoal.DISABLED)
                .beforeStarting(() -> disabledAlert.set(true))
                .andThen(Commands.idle())
                .finallyDo(() -> {
                    goal = TurretGoal.IDLE;
                    disabledAlert.set(false);
                })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .ignoringDisable(true)
                .withName("Disable Turret");
    }

    public Command manualOverride() {
        return setGoal(TurretGoal.MANUAL_OVERRIDE)
                .beforeStarting(() -> manualOverrideAlert.set(true))
                .andThen(Commands.idle())
                .finallyDo(() -> {
                    if (goal != TurretGoal.DISABLED) {
                        goal = TurretGoal.IDLE;
                        io.stopFlywheel();
                        io.stopHood();
                        io.stopTurn();
                    }
                    manualOverrideAlert.set(false);
                })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("Turret Manual Override");
    }

    public Command manualScore() {
        return Commands.startEnd(
                        () -> {
                            io.setFlywheelSpeed(FLYWHEEL_SCORING_OVERRIDE.times(flywheelFudgeFactor));
                            io.setHoodAngle(HOOD_SCORING_OVERRIDE);
                            io.setTurnSetpoint(Degrees.of(turnTrimDeg), RadiansPerSecond.zero());
                        },
                        () -> {
                            io.stopFlywheel();
                            io.setHoodAngle(MIN_HOOD_ANGLE);
                            io.setTurnSetpoint(Degrees.of(turnTrimDeg), RadiansPerSecond.zero());
                        })
                .onlyIf(() -> goal == TurretGoal.MANUAL_OVERRIDE)
                .withName("Turret Manual Score");
    }

    public Command manualPass() {
        return Commands.startEnd(
                        () -> {
                            io.setFlywheelSpeed(FLYWHEEL_PASSING_OVERRIDE.times(flywheelFudgeFactor));
                            io.setHoodAngle(HOOD_PASSING_OVERRIDE);
                            io.setTurnSetpoint(Degrees.of(turnTrimDeg), RadiansPerSecond.zero());
                        },
                        () -> {
                            io.stopFlywheel();
                            io.setHoodAngle(MIN_HOOD_ANGLE);
                            io.setTurnSetpoint(Degrees.of(turnTrimDeg), RadiansPerSecond.zero());
                        })
                .onlyIf(() -> goal == TurretGoal.MANUAL_OVERRIDE)
                .withName("Turret Manual Pass");
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        Logger.recordOutput(
                "Turret/Current Command",
                this.getCurrentCommand() == null
                        ? "None"
                        : this.getCurrentCommand().getName());

        Pose2d pose = poseSupplier.get();

        switch (goal) {
            case SCORING:
                calculateShot(pose);
                break;
            case EVIL:
                calculateShot(pose);
                break;
            case PASSING:
                calculateShot(pose);
                setTarget(getPassingTarget(pose), false);
                break;
            case TUNING:
                io.setFlywheelSpeed(RPM.of(tuningFlywheelSpeed.get()));
                io.setHoodAngle(Degrees.of(tuningHoodAngle.get()));
                io.setTurnSetpoint(
                        TurretCalculator.calculateAzimuthAngle(pose, currentTarget, inputs.turnPosition)
                                .plus(Degrees.of(turnTrimDeg).plus(BASE_TRIM)),
                        RPM.zero());
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
        // ChassisSpeeds requestedFieldSpeeds = requestedRobotSpeedsSupplier.get();

        ShotData calculatedShot;
        if (Constants.CURRENT_MODE == Mode.REAL) {
            calculatedShot = TurretCalculator.iterativeMovingShotFromMap(
                    robotPose,
                    fieldSpeeds,
                    currentTarget,
                    LOOKAHEAD_ITERATIONS,
                    goal == TurretGoal.SCORING ? SHOT_MAP : PASSING_MAP,
                    goal == TurretGoal.SCORING ? TOF_MAP : PASS_TOF_MAP,
                    tofFudgeFactor,
                    pitchSupplier.get(),
                    rollSupplier.get());
        } else {
            calculatedShot = TurretCalculator.iterativeMovingShotFromFunnelClearance(
                    robotPose, fieldSpeeds, currentTarget, LOOKAHEAD_ITERATIONS);
        }
        Angle azimuthAngle =
                TurretCalculator.calculateAzimuthAngle(robotPose, calculatedShot.target(), inputs.turnPosition);

        Translation2d robotToTarget = calculatedShot.target().toTranslation2d().minus(robotPose.getTranslation());
        Time timeOfFlight =
                Seconds.of((goal == TurretGoal.SCORING ? TOF_MAP : PASS_TOF_MAP).get(robotToTarget.getNorm()));
        AngularVelocity velocitySetpoint = getVelocitySetpoint(robotToTarget, fieldSpeeds, timeOfFlight);

        io.setTurnSetpoint(
                azimuthAngle
                        .plus(velocitySetpoint.times(Seconds.of(0.02)))
                        .plus(Degrees.of(turnTrimDeg).plus(BASE_TRIM)),
                velocitySetpoint);
        io.setHoodAngle(calculatedShot.getHoodAngle());
        io.setFlywheelSpeed(calculatedShot.getAngularExitVelocity().times(flywheelFudgeFactor));

        Logger.recordOutput("Turret/Shot", calculatedShot);
        Logger.recordOutput("Turret/PosError", inputs.turnSetpoint.minus(inputs.turnPosition));
        Logger.recordOutput("Turret/VelError", velocitySetpoint.minus(inputs.turnVelocity));
    }

    public Command resetTurnEncoder() {
        return Commands.runOnce(io::resetTurnEncoder).ignoringDisable(true).withName("Reset Turret Encoder");
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

    public Command moveToZero() {
        return this.runOnce(() -> {
            io.setHoodAngle(MIN_HOOD_ANGLE);
            io.stopFlywheel();
            io.setTurnSetpoint(Degrees.of(turnTrimDeg), RadiansPerSecond.zero());
        });
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
        EVIL,
        MANUAL_OVERRIDE,
        DISABLED
    }
}
