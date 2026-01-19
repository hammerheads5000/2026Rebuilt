// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.TurretConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;
import frc.robot.util.TunableControls.TunableProfiledController;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;

    private final TunableProfiledController turnController;
    private final TunableProfiledController hoodController;
    private final TunableProfiledController flywheelController;
    private final TunableProfiledController shootController;

    @AutoLogOutput
    Translation3d currentTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? FieldConstants.HUB_BLUE
            : FieldConstants.HUB_RED;

    private final TurretVisualizer turretVisualizer;

    public Turret(TurretIO io, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.io = io;
        this.inputs = new TurretIOInputsAutoLogged();
        this.poseSupplier = poseSupplier;
        this.fieldSpeedsSupplier = fieldSpeedsSupplier;

        turnController = new TunableProfiledController(TURN_TUNABLE_CONSTANTS);
        hoodController = new TunableProfiledController(HOOD_TUNABLE_CONSTANTS);
        flywheelController = new TunableProfiledController(FLYWHEEL_TUNABLE_CONSTANTS);
        shootController = new TunableProfiledController(SHOOT_TUNABLE_CONSTANTS);

        turretVisualizer = new TurretVisualizer(
                () -> new Pose3d(poseSupplier
                                .get()
                                .rotateAround(
                                        poseSupplier.get().getTranslation(), new Rotation2d(turnController.getGoal())))
                        .transformBy(ROBOT_TO_TURRET_TRANSFORM),
                fieldSpeedsSupplier);

        SmartDashboard.putData(this.runOnce(() -> turretVisualizer.launchFuel(
                        TurretCalculator.angularToLinearVelocity(
                                RadiansPerSecond.of(flywheelController.getGoal()), FLYWHEEL_RADIUS),
                        Radians.of(hoodController.getGoal())))
                .withName("Launch Fuel"));

        SmartDashboard.putData("Turret/Turn Controller", turnController.getProfiledPIDController());
        SmartDashboard.putData("Turret/Hood Controller", hoodController.getProfiledPIDController());
        SmartDashboard.putData("Turret/Flywheel Controller", flywheelController.getProfiledPIDController());
        SmartDashboard.putData("Turret/Shoot Controller", shootController.getProfiledPIDController());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        currentTarget = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? FieldConstants.HUB_BLUE
                : FieldConstants.HUB_RED;

        Pose2d robot = poseSupplier.get();
        ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

        ShotData calculatedShot = TurretCalculator.iterativeMovingShotFromFunnelClearance(
                robot, fieldSpeeds, currentTarget, LOOKAHEAD_ITERATIONS);
        Angle azimuthAngle = TurretCalculator.calculateAzimuthAngle(robot, calculatedShot.target());
        AngularVelocity azimuthVelocity = RadiansPerSecond.of(-fieldSpeeds.omegaRadiansPerSecond);
        turnController.setGoal(azimuthAngle.in(Radians), azimuthVelocity.in(RadiansPerSecond));
        hoodController.setGoal(calculatedShot.getHoodAngle().in(Radians));
        flywheelController.setGoal(
                TurretCalculator.linearToAngularVelocity(calculatedShot.getExitVelocity(), FLYWHEEL_RADIUS)
                        .in(RadiansPerSecond));
        shootController.setGoal(TurretCalculator.linearToAngularVelocity(calculatedShot.getExitVelocity(), SHOOT_RADIUS)
                .in(RadiansPerSecond));

        Voltage turnVoltage = Volts.of(turnController.calculate(inputs.turnPosition.in(Radians)));
        Voltage hoodVoltage = Volts.of(hoodController.calculate(inputs.hoodPosition.in(Radians)));
        Voltage flywheelVoltage = Volts.of(flywheelController.calculate(inputs.flywheelSpeed.in(RadiansPerSecond)));
        Voltage shootVoltage = Volts.of(shootController.calculate(inputs.shootSpeed.in(RadiansPerSecond)));

        io.setTurnOutput(turnVoltage);
        io.setHoodOutput(hoodVoltage);
        io.setFlywheelOutput(flywheelVoltage);
        io.setShootOutput(shootVoltage);
        turretVisualizer.updateFuel(calculatedShot.getExitVelocity(), calculatedShot.getHoodAngle());
        turretVisualizer.update3dPose(azimuthAngle);

        Logger.recordOutput("Turret/Shot", calculatedShot);
        Logger.recordOutput("Turret/Turn Voltage", turnVoltage);
        Logger.recordOutput("Turret/Hood Voltage", hoodVoltage);
        Logger.recordOutput("Turret/Flywheel Voltage", flywheelVoltage);
        Logger.recordOutput("Turret/Shoot Voltage", shootVoltage);
        Logger.recordOutput("Turret/Azimuth Angle", azimuthAngle);
        Logger.recordOutput(
                "Turret/Azimuth Visualizer",
                new Pose2d(robot.getTranslation(), new Rotation2d(azimuthAngle).plus(robot.getRotation())));
    }
}
