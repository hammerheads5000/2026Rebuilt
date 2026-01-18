// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.InchesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.BASE_VEL;
import static frc.robot.Constants.IntakeConstants.VEL_MULTIPLIER;
import static frc.robot.Constants.IntakeConstants.VEL_POWER;
import static frc.robot.Constants.TurretConstants.*;

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
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
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
                        angularToLinearVelocity(RadiansPerSecond.of(flywheelController.getGoal()), FLYWHEEL_RADIUS),
                        Radians.of(hoodController.getGoal())))
                .withName("Launch Fuel"));

        SmartDashboard.putData("Turret/Turn Controller", turnController.getProfiledPIDController());
        SmartDashboard.putData("Turret/Hood Controller", hoodController.getProfiledPIDController());
        SmartDashboard.putData("Turret/Flywheel Controller", flywheelController.getProfiledPIDController());
        SmartDashboard.putData("Turret/Shoot Controller", shootController.getProfiledPIDController());
    }

    private Distance getDistanceToTarget(Pose2d robot, Translation3d target) {
        Pose2d turret = new Pose3d(robot).transformBy(ROBOT_TO_TURRET_TRANSFORM).toPose2d();
        Translation2d target2d = target.toTranslation2d();
        return Meters.of(Math.hypot(turret.getX() - target2d.getX(), turret.getY() - target2d.getY()));
    }

    private LinearVelocity calculateLinearVelocity(Distance distanceToTarget) {
        double velocity =
                BASE_VEL.in(InchesPerSecond) + VEL_MULTIPLIER * Math.pow(distanceToTarget.in(Inches), VEL_POWER);
        return InchesPerSecond.of(velocity);
    }

    // see https://www.desmos.com/geometry/l4edywkmha
    private Angle calculateAngle(Pose2d robot, LinearVelocity velocity, Translation3d target) {
        double g = MetersPerSecondPerSecond.of(9.81).in(InchesPerSecondPerSecond);
        double vel = velocity.in(InchesPerSecond);
        double x_dist = getDistanceToTarget(robot, target).in(Inches);
        double y_dist = target.getMeasureZ()
                .minus(ROBOT_TO_TURRET_TRANSFORM.getMeasureZ())
                .in(Inches);
        double angle = Math.atan(
                ((vel * vel) + Math.sqrt(Math.pow(vel, 4) - g * (g * x_dist * x_dist + 2 * y_dist * vel * vel)))
                        / (g * x_dist));
        return Radians.of(angle);
    }

    private Time calculateTimeToHit(Pose2d robot, LinearVelocity velocity, Angle hoodAngle, Distance distance) {
        double vel = velocity.in(MetersPerSecond);
        double angle = hoodAngle.in(Radians);
        double x_dist = distance.in(Meters);
        return Seconds.of(x_dist / (vel * Math.cos(angle)));
    }

    private AngularVelocity linearToAngularVelocity(LinearVelocity vel, Distance radius) {
        return RadiansPerSecond.of(vel.in(MetersPerSecond) / radius.in(Meters));
    }

    private LinearVelocity angularToLinearVelocity(AngularVelocity vel, Distance radius) {
        return MetersPerSecond.of(vel.in(RadiansPerSecond) * radius.in(Meters));
    }

    private Angle calculateAzimuthAngle(Pose2d turret, Translation3d target) {
        Translation2d turretTranslation = new Pose3d(turret)
                .transformBy(ROBOT_TO_TURRET_TRANSFORM)
                .toPose2d()
                .getTranslation();

        Translation2d direction = target.toTranslation2d().minus(turretTranslation);

        return Radians.of(MathUtil.inputModulus(
                direction.getAngle().minus(turret.getRotation()).getRadians(), 0, 2 * Math.PI));
    }

    private Translation3d predictTargetPos(Pose2d turret, ChassisSpeeds fieldSpeeds, Time timeToHit) {
        double predictedX = currentTarget.getX() - fieldSpeeds.vxMetersPerSecond * timeToHit.in(Seconds);
        double predictedY = currentTarget.getY() - fieldSpeeds.vyMetersPerSecond * timeToHit.in(Seconds);
        double predictedZ = currentTarget.getZ();

        return new Translation3d(predictedX, predictedY, predictedZ);
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

        // Make initial guess trajectory assuming robot is stationary
        Distance distanceToTarget = getDistanceToTarget(robot, currentTarget);
        LinearVelocity vel = calculateLinearVelocity(distanceToTarget);
        Angle hoodAngle = calculateAngle(robot, vel, currentTarget);

        Time timeToHit = calculateTimeToHit(robot, vel, hoodAngle, distanceToTarget);
        Translation3d predictedTarget = predictTargetPos(robot, fieldSpeeds, timeToHit);
        hoodAngle = calculateAngle(robot, vel, predictedTarget);
        distanceToTarget = getDistanceToTarget(robot, predictedTarget);
        // vel = calculateLinearVelocity(distanceToTarget);
        // Predict target position and recalculate trajectory
        for (int i = 0; i < 3; i++) {
            timeToHit = calculateTimeToHit(robot, vel, hoodAngle, distanceToTarget);
            predictedTarget = predictTargetPos(robot, fieldSpeeds, timeToHit);
            hoodAngle = calculateAngle(robot, vel, predictedTarget);
            distanceToTarget = getDistanceToTarget(robot, predictedTarget);
        }
        Angle azimuthAngle = calculateAzimuthAngle(robot, predictedTarget);
        AngularVelocity azimuthVelocity = RadiansPerSecond.of(-fieldSpeeds.omegaRadiansPerSecond);
        turnController.setGoal(azimuthAngle.in(Radians), azimuthVelocity.in(RadiansPerSecond));
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
        Logger.recordOutput("Turret/Azimuth Angle", azimuthAngle);
        Logger.recordOutput(
                "Turret/Azimuth Visualizer",
                new Pose2d(robot.getTranslation(), new Rotation2d(azimuthAngle).plus(robot.getRotation())));
        Logger.recordOutput("Turret/Predicted Target", predictedTarget);
    }
}
