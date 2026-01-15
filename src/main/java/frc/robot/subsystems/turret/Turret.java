// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.TurretConstants.*;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.TunableControls.TunableProfiledController;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs;
    private final Supplier<Pose2d> poseSupplier;

    private final TunableProfiledController turnController;
    private final TunableProfiledController hoodController;
    private final TunableProfiledController flywheelController;
    private final TunableProfiledController shootController;

    Translation3d target = FieldConstants.HUB;

    public Turret(TurretIO io, Supplier<Pose2d> poseSupplier) {
        this.io = io;
        this.inputs = new TurretIOInputsAutoLogged();
        this.poseSupplier = poseSupplier;

        turnController = new TunableProfiledController(TURN_TUNABLE_CONSTANTS);
        hoodController = new TunableProfiledController(HOOD_TUNABLE_CONSTANTS);
        flywheelController = new TunableProfiledController(FLYWHEEL_TUNABLE_CONSTANTS);
        shootController = new TunableProfiledController(SHOOT_TUNABLE_CONSTANTS);
    }

    private Distance getDistanceToHub() {
        Pose2d turret = new Pose3d(poseSupplier.get())
                .transformBy(ROBOT_TO_TURRET_TRANSFORM)
                .toPose2d();
        Translation2d hub = target.toTranslation2d();
        return Meters.of(Math.hypot(turret.getX() - hub.getX(), turret.getY() - hub.getY()));
    }

    // see https://www.desmos.com/calculator/ezjqolho6g
    private Pair<LinearVelocity, Angle> calculateShot() {
        double x_dist = getDistanceToHub().in(Meters);
        double y_dist = target.getZ() - ROBOT_TO_TURRET_TRANSFORM.getZ();
        double g = 9.81;
        double r = FieldConstants.FUNNEL_RADIUS.in(Meters);
        double h = FieldConstants.FUNNEL_HEIGHT.in(Meters);
        double A1 = x_dist * x_dist;
        double B1 = x_dist;
        double D1 = y_dist;
        double A2 = -x_dist * x_dist + (x_dist - r) * (x_dist - r);
        double B2 = r;
        double D2 = h;
        double Bm = -B2 / B1;
        double A3 = Bm * A1 + A2;
        double D3 = Bm * D1 + D2;
        double a = D3 / A3;
        double b = (D1 - A1 * a) / B1;
        double theta = Math.atan(b);
        double v0 = Math.sqrt(-g / (2 * a * (Math.cos(theta)) * (Math.cos(theta))));
        return Pair.of(MetersPerSecond.of(v0), Radians.of(theta));
    }

    private AngularVelocity linearToAngularVelocity(LinearVelocity vel, Distance radius) {
        return RadiansPerSecond.of(vel.in(MetersPerSecond) / radius.in(Meters));
    }

    private Angle calculateAzimuthAngle() {
        Pose2d robot = poseSupplier.get();
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

        var shot = calculateShot();
        LinearVelocity vel = shot.getFirst();
        Angle hoodAngle = shot.getSecond();
        Angle azimuthAngle = calculateAzimuthAngle();
        turnController.setGoal(azimuthAngle.in(Radians));
        hoodController.setGoal(hoodAngle.in(Radians));
        flywheelController.setGoal(linearToAngularVelocity(vel, FLYWHEEL_RADIUS).in(RadiansPerSecond));
        shootController.setGoal(linearToAngularVelocity(vel, SHOOT_RADIUS).in(RadiansPerSecond));

        io.setTurnOutput(Volts.of(turnController.calculate(inputs.turnPosition.in(Radians))));
        io.setHoodOutput(Volts.of(hoodController.calculate(inputs.hoodPosition.in(Radians))));
        io.setFlywheelOutput(Volts.of(flywheelController.calculate(inputs.flywheelSpeed.in(RadiansPerSecond))));
        io.setShootOutput(Volts.of(shootController.calculate(inputs.shootSpeed.in(RadiansPerSecond))));
    }
}
