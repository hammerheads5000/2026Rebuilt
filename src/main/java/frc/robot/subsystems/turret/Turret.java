// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.TurretConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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

        

        turretVisualizer = new TurretVisualizer(
                () -> new Pose3d(poseSupplier
                                .get()
                                .rotateAround(poseSupplier.get().getTranslation(), new Rotation2d(inputs.turnPosition)))
                        .transformBy(ROBOT_TO_TURRET_TRANSFORM),
                fieldSpeedsSupplier);

        // this.setDefaultCommand(turretVisualizer.repeatedlyLaunchFuel(
        //         () -> TurretCalculator.angularToLinearVelocity(inputs.flywheelSpeed, FLYWHEEL_RADIUS),
        //         () -> inputs.hoodPosition,
        //         this));

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

    public boolean simAbleToIntake() {
        return turretVisualizer.canIntake();
    }

    public void simIntake() {
        turretVisualizer.intakeFuel();
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
        io.setTurnSetpoint(azimuthAngle, azimuthVelocity);
        io.setHoodAngle(calculatedShot.getHoodAngle());
        io.setFlywheelSpeed(
                TurretCalculator.linearToAngularVelocity(calculatedShot.getExitVelocity(), FLYWHEEL_RADIUS));

        turretVisualizer.update3dPose(inputs.turnPosition);

        Logger.recordOutput("Turret/Shot", calculatedShot);

        turnController.logData("Turret/TurnController");
        hoodController.logData("Turret/HoodController");
        flywheelController.logData("Turret/FlywheelController");
        shootController.logData("Turret/ShootController");
    }

    @Override
    public void simulationPeriodic() {
        turretVisualizer.updateFuel(
                TurretCalculator.angularToLinearVelocity(inputs.flywheelSpeed, FLYWHEEL_RADIUS), inputs.hoodPosition);
    }
}
