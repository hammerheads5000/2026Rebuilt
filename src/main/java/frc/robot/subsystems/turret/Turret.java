// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.TurretConstants.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.TunableControls.TunableProfiledController;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs;
    private final Supplier<Pose2d> poseSupplier;

    private final TunableProfiledController turnController;
    private final TunableProfiledController hoodController;
    private final TunableProfiledController flywheelController;
    private final TunableProfiledController shootController;

    @AutoLogOutput
    Translation3d target = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            ? FieldConstants.HUB_BLUE
            : FieldConstants.HUB_RED;

    private final TurretVisualizer turretVisualizer;

    public Turret(TurretIO io, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.io = io;
        this.inputs = new TurretIOInputsAutoLogged();
        this.poseSupplier = poseSupplier;

        turnController = new TunableProfiledController(TURN_TUNABLE_CONSTANTS);
        hoodController = new TunableProfiledController(HOOD_TUNABLE_CONSTANTS);
        flywheelController = new TunableProfiledController(FLYWHEEL_TUNABLE_CONSTANTS);
        shootController = new TunableProfiledController(SHOOT_TUNABLE_CONSTANTS);

        turretVisualizer = new TurretVisualizer(
                () -> new Pose3d(poseSupplier
                                .get()
                                .rotateAround(poseSupplier.get().getTranslation(), new Rotation2d(inputs.turnPosition)))
                        .transformBy(ROBOT_TO_TURRET_TRANSFORM),
                chassisSpeedsSupplier);

        SmartDashboard.putData(this.runOnce(() -> turretVisualizer.launchFuel(
                        angularToLinearVelocity(inputs.flywheelSpeed, FLYWHEEL_RADIUS), inputs.hoodPosition))
                .withName("Launch Fuel"));
    }

    private Distance getDistanceToHub(Pose2d robot) {
        Pose2d turret = new Pose3d(robot).transformBy(ROBOT_TO_TURRET_TRANSFORM).toPose2d();
        Translation2d hub = target.toTranslation2d();
        return Meters.of(Math.hypot(turret.getX() - hub.getX(), turret.getY() - hub.getY()));
    }

    // see https://www.desmos.com/calculator/ezjqolho6g
    private Pair<LinearVelocity, Angle> calculateShot(Pose2d robot) {
        double x_dist = getDistanceToHub(robot).in(Inches);
        double y_dist = target.getMeasureZ().minus(ROBOT_TO_TURRET_TRANSFORM.getMeasureZ()).in(Inches);
        double g = 316;
        double r = FieldConstants.FUNNEL_RADIUS.in(Inches);
        double h = FieldConstants.FUNNEL_HEIGHT.plus(DISTANCE_ABOVE_FUNNEL).in(Inches);
        double A1 = x_dist * x_dist;
        double B1 = x_dist;
        double D1 = y_dist;
        double A2 = -x_dist * x_dist + (x_dist - r) * (x_dist - r);
        double B2 = -r;
        double D2 = h;
        double Bm = -B2 / B1;
        double A3 = Bm * A1 + A2;
        double D3 = Bm * D1 + D2;
        double a = D3 / A3;
        double b = (D1 - A1 * a) / B1;
        double theta = Math.atan(b);
        double v0 = Math.sqrt(-g / (2 * a * (Math.cos(theta)) * (Math.cos(theta))));
        return Pair.of(InchesPerSecond.of(v0), Radians.of(theta));
    }

    private AngularVelocity linearToAngularVelocity(LinearVelocity vel, Distance radius) {
        return RadiansPerSecond.of(vel.in(MetersPerSecond) / radius.in(Meters));
    }

    private LinearVelocity angularToLinearVelocity(AngularVelocity vel, Distance radius) {
        return MetersPerSecond.of(vel.in(RadiansPerSecond) * radius.in(Meters));
    }

    private Angle calculateAzimuthAngle(Pose2d robot) {
        Translation2d turret = new Pose3d(robot)
                .transformBy(ROBOT_TO_TURRET_TRANSFORM)
                .toPose2d()
                .getTranslation();

        Translation2d direction = target.toTranslation2d().minus(turret);
        return direction.getAngle().minus(robot.getRotation()).getMeasure();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        target = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                ? FieldConstants.HUB_BLUE
                : FieldConstants.HUB_RED;

        Pose2d robot = poseSupplier.get();

        var shot = calculateShot(robot);
        LinearVelocity vel = shot.getFirst();
        Angle hoodAngle = shot.getSecond();
        Angle azimuthAngle = calculateAzimuthAngle(robot);
        turnController.setGoal(azimuthAngle.in(Radians));
        hoodController.setGoal(hoodAngle.in(Radians));
        flywheelController.setGoal(linearToAngularVelocity(vel, FLYWHEEL_RADIUS).in(RadiansPerSecond));
        shootController.setGoal(linearToAngularVelocity(vel, SHOOT_RADIUS).in(RadiansPerSecond));

        Voltage turnVoltage = Volts.of(turnController.calculate(inputs.turnPosition.in(Radians)));
        Voltage hoodVoltage = Volts.of(hoodController.calculate(inputs.hoodPosition.in(Radians)));
        Voltage flywheelVoltage = Volts.of(flywheelController.calculate(inputs.flywheelSpeed.in(RadiansPerSecond)));
        Voltage shootVoltage = Volts.of(shootController.calculate(inputs.shootSpeed.in(RadiansPerSecond)));

        io.setTurnOutput(turnVoltage);
        io.setHoodOutput(hoodVoltage);
        io.setFlywheelOutput(flywheelVoltage);
        io.setShootOutput(shootVoltage);
        turretVisualizer.updateFuel(vel, hoodAngle);

        Logger.recordOutput("Turret/Velocity", vel);
        Logger.recordOutput("Turret/Hood Angle", hoodAngle);
        Logger.recordOutput("Turret/Turn Voltage", turnVoltage);
        Logger.recordOutput("Turret/Hood Voltage", hoodVoltage);
        Logger.recordOutput("Turret/Flywheel Voltage", flywheelVoltage);
        Logger.recordOutput("Turret/Shoot Voltage", shootVoltage);
        Logger.recordOutput(
                "Turret/Azimuth Angle",
                new Pose2d(robot.getTranslation(), new Rotation2d(azimuthAngle).plus(robot.getRotation())));
    }
}
