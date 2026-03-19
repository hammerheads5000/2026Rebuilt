// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.Constants.IntakeConstants.DISTANCE_INTO_WALL;
import static frc.robot.Constants.IntakeConstants.MAX_EXTENSION;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.SlewRateLimiter2d;
import frc.robot.util.TunableControls.TunablePIDController;
import frc.robot.util.Zones;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Default drive command to run that drives based on controller input */
public class TeleopDrive extends Command {
    private final Drive drive;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier omegaSupplier;
    private final BooleanSupplier leftIntakeDeployed;
    private final BooleanSupplier rightIntakeDeployed;
    private final SlewRateLimiter2d driveLimiter;
    private int flipFactor = 1; // 1 for normal, -1 for flipped

    private LinearVelocity maxDriveSpeed = SwerveConstants.DEFAULT_DRIVE_SPEED;
    private AngularVelocity maxRotSpeed = SwerveConstants.DEFAULT_ROT_SPEED;

    @AutoLogOutput
    private final Trigger inTrenchZoneTrigger;

    @AutoLogOutput
    private final Trigger inBumpZoneTrigger;

    @AutoLogOutput
    private final Trigger inTowerZoneTrigger;

    @AutoLogOutput
    private boolean wallAvoidance = true;

    private final TunablePIDController translationController =
            new TunablePIDController(SwerveConstants.TRANSLATION_CONSTANTS);
    private final TunablePIDController rotationController =
            new TunablePIDController(SwerveConstants.ROTATION_CONSTANTS);

    @AutoLogOutput
    private DriveMode currentDriveMode = DriveMode.NORMAL;

    /** Creates a new TeleopDrive. */
    public TeleopDrive(
            Drive drive,
            CommandXboxController controller,
            BooleanSupplier leftIntakeDeployed,
            BooleanSupplier rightIntakeDeployed) {
        this.drive = drive;
        this.xSupplier = () -> -controller.getLeftY() * flipFactor;
        this.ySupplier = () -> -controller.getLeftX() * flipFactor;
        this.omegaSupplier = () -> -controller.getRightX();
        this.driveLimiter = new SlewRateLimiter2d(SwerveConstants.MAX_TELEOP_ACCEL.in(MetersPerSecondPerSecond));
        this.leftIntakeDeployed = leftIntakeDeployed;
        this.rightIntakeDeployed = rightIntakeDeployed;

        inTrenchZoneTrigger = Zones.TRENCH_ZONES
                .willContain(drive::getPose, drive::getFieldSpeeds, SwerveConstants.TRENCH_ALIGN_TIME)
                .debounce(0.1);

        inBumpZoneTrigger = Zones.BUMP_ZONES
                .willContain(drive::getPose, drive::getFieldSpeeds, SwerveConstants.BUMP_ALIGN_TIME)
                .debounce(0.1);

        inTowerZoneTrigger = Zones.TOWER_ZONES.contains(drive::getPose).debounce(0.1);

        inTrenchZoneTrigger.onTrue(updateDriveMode(DriveMode.TRENCH_LOCK));
        inBumpZoneTrigger.onTrue(updateDriveMode(DriveMode.BUMP_LOCK));
        inTrenchZoneTrigger.or(inBumpZoneTrigger).or(inTowerZoneTrigger).onFalse(updateDriveMode(DriveMode.NORMAL));
        inTowerZoneTrigger.and(() -> wallAvoidance).onTrue(updateDriveMode(DriveMode.TOWER_LOCK));

        addRequirements(drive);
    }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), ControllerConstants.CONTROLLER_DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    private Distance getTrenchY() {
        Pose2d robotPose = drive.getPose();
        if (robotPose.getMeasureY().gte(FieldConstants.FIELD_WIDTH.div(2))) {
            return FieldConstants.FIELD_WIDTH.minus(FieldConstants.TRENCH_CENTER);
        }
        return FieldConstants.TRENCH_CENTER;
    }

    private Distance getTowerX() {
        Pose2d robotPose = drive.getPose();
        if (robotPose.getMeasureX().gte(FieldConstants.FIELD_LENGTH.div(2))) {
            return FieldConstants.FIELD_LENGTH.minus(FieldConstants.TOWER_CENTER_X);
        }
        return FieldConstants.TOWER_CENTER_X;
    }

    private Rotation2d getTrenchLockAngle() {
        if (Math.abs(MathUtil.inputModulus(drive.getRotation().getDegrees() - 90, -180, 180)) < 90) {
            return Rotation2d.kCCW_90deg;
        } else {
            return Rotation2d.kCW_90deg;
        }
    }

    private Rotation2d getTowerLockAngle() {
        if (Math.abs(MathUtil.inputModulus(drive.getRotation().getDegrees(), -180, 180)) < 90) {
            return Rotation2d.kZero;
        } else {
            return Rotation2d.k180deg;
        }
    }

    private Rotation2d getBumpLockAngle() {
        for (int i = -135; i < 180; i += 90) {
            if (Math.abs(MathUtil.inputModulus(drive.getRotation().getDegrees() - i, -180, 180)) <= 45) {
                return Rotation2d.fromDegrees(i);
            }
        }
        return Rotation2d.kZero;
    }

    private Command updateDriveMode(DriveMode driveMode) {
        return Commands.runOnce(() -> currentDriveMode = driveMode);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        flipFactor = DriverStation.getAlliance().isPresent()
                        && DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                ? -1
                : 1;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());
        linearVelocity = linearVelocity.times(maxDriveSpeed.in(MetersPerSecond));
        linearVelocity = driveLimiter.calculate(linearVelocity);

        double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), ControllerConstants.CONTROLLER_DEADBAND);
        omega = Math.copySign(omega * omega, omega); // square for more precise rotation control
        AngularVelocity angularVelocity = maxRotSpeed.times(omega);

        Logger.recordOutput(
                "TeleopDrive/Raw Speeds",
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.getMeasureX().per(Second),
                        linearVelocity.getMeasureY().per(Second),
                        angularVelocity,
                        drive.getRotation()));

        switch (currentDriveMode) {
            case NORMAL:
                if (wallAvoidance) {
                    linearVelocity = applyWallAvoidance(linearVelocity);
                }
                break;
            case TRENCH_LOCK:
                translationController.setSetpoint(getTrenchY().in(Meters));
                double yVel = translationController.calculate(drive.getPose().getY());
                if (translationController.atSetpoint()) {
                    yVel = 0;
                }
                rotationController.setSetpoint(getTrenchLockAngle().getRadians());
                double rotSpeedToStraight =
                        rotationController.calculate(drive.getRotation().getRadians());
                if (rotationController.atSetpoint()) {
                    rotSpeedToStraight = 0;
                }
                linearVelocity = new Translation2d(linearVelocity.getX(), yVel);
                angularVelocity = RadiansPerSecond.of(rotSpeedToStraight);
                break;
            case BUMP_LOCK:
                rotationController.setSetpoint(getBumpLockAngle().getRadians());
                double rotSpeedToDiagonal =
                        rotationController.calculate(drive.getRotation().getRadians());
                if (rotationController.atSetpoint()) {
                    rotSpeedToDiagonal = 0;
                }
                angularVelocity = RadiansPerSecond.of(rotSpeedToDiagonal);
                break;
            case TOWER_LOCK:
                translationController.setSetpoint(getTowerX().in(Meters));
                double xVel = translationController.calculate(drive.getPose().getX());
                if (translationController.atSetpoint()) {
                    xVel = 0;
                }
                rotationController.setSetpoint(getTowerLockAngle().getRadians());
                double rotSpeedToForward =
                        rotationController.calculate(drive.getRotation().getRadians());
                linearVelocity = new Translation2d(xVel, linearVelocity.getY());
                angularVelocity = RadiansPerSecond.of(rotSpeedToForward);
                break;
        }

        Logger.recordOutput(
                "TeleopDrive/Adjusted Speeds",
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        linearVelocity.getMeasureX().per(Second),
                        linearVelocity.getMeasureY().per(Second),
                        angularVelocity,
                        drive.getRotation()));

        drive.driveFieldCentric(
                MetersPerSecond.of(linearVelocity.getX()), MetersPerSecond.of(linearVelocity.getY()), angularVelocity);
    }

    private void setDriveSpeed(LinearVelocity speed) {
        maxDriveSpeed = speed;
    }

    private void setRotSpeed(AngularVelocity speed) {
        maxRotSpeed = speed;
    }

    public Command speedUpCommand() {
        return Commands.startEnd(
                        () -> {
                            setDriveSpeed(SwerveConstants.FAST_DRIVE_SPEED);
                            setRotSpeed(SwerveConstants.FAST_ROT_SPEED);
                        },
                        () -> {
                            setDriveSpeed(SwerveConstants.DEFAULT_DRIVE_SPEED);
                            setRotSpeed(SwerveConstants.DEFAULT_ROT_SPEED);
                        })
                .withName("Speed Up");
    }

    public Command slowDownCommand() {
        return Commands.startEnd(
                        () -> {
                            setDriveSpeed(SwerveConstants.SLOW_DRIVE_SPEED);
                            setRotSpeed(SwerveConstants.SLOW_ROT_SPEED);
                            driveLimiter.setRateLimit(SwerveConstants.SLOW_TELEOP_ACCEL.in(MetersPerSecondPerSecond));
                        },
                        () -> {
                            setDriveSpeed(SwerveConstants.DEFAULT_DRIVE_SPEED);
                            setRotSpeed(SwerveConstants.DEFAULT_ROT_SPEED);
                            driveLimiter.setRateLimit(SwerveConstants.MAX_TELEOP_ACCEL.in(MetersPerSecondPerSecond));
                        })
                .withName("Slow Down");
    }

    public Command disableWallAvoidance() {
        return Commands.startEnd(() -> wallAvoidance = false, () -> wallAvoidance = true)
                .withName("Disable Wall Avoidance");
    }

    public Translation2d applyWallAvoidance(Translation2d vel) {
        Translation2d frontLeftCorner = new Translation2d(
                Dimensions.FULL_LENGTH.div(2),
                Dimensions.FULL_WIDTH.div(2).plus(leftIntakeDeployed.getAsBoolean() ? MAX_EXTENSION : Meters.of(0)));
        Translation2d backLeftCorner = new Translation2d(
                Dimensions.FULL_LENGTH.div(2).unaryMinus(),
                Dimensions.FULL_WIDTH.div(2).plus(leftIntakeDeployed.getAsBoolean() ? MAX_EXTENSION : Meters.of(0)));
        Translation2d frontRightCorner = new Translation2d(
                        Dimensions.FULL_LENGTH.div(2),
                        Dimensions.FULL_WIDTH
                                .div(2)
                                .plus(rightIntakeDeployed.getAsBoolean() ? MAX_EXTENSION : Meters.of(0)))
                .unaryMinus();
        Translation2d backRightCorner = new Translation2d(
                        Dimensions.FULL_LENGTH.div(2).unaryMinus(),
                        Dimensions.FULL_WIDTH
                                .div(2)
                                .plus(rightIntakeDeployed.getAsBoolean() ? MAX_EXTENSION : Meters.of(0)))
                .unaryMinus();

        Pose2d pose = drive.getPose();
        frontLeftCorner = pose.transformBy(new Transform2d(frontLeftCorner, Rotation2d.kZero))
                .getTranslation();
        backLeftCorner = pose.transformBy(new Transform2d(backLeftCorner, Rotation2d.kZero))
                .getTranslation();
        frontRightCorner = pose.transformBy(new Transform2d(frontRightCorner, Rotation2d.kZero))
                .getTranslation();
        backRightCorner = pose.transformBy(new Transform2d(backRightCorner, Rotation2d.kZero))
                .getTranslation();

        double maxX = Math.max(
                Math.max(Math.max(frontLeftCorner.getX(), backLeftCorner.getX()), frontRightCorner.getX()),
                backRightCorner.getX());
        double minX = Math.min(
                Math.min(Math.min(frontLeftCorner.getX(), backLeftCorner.getX()), frontRightCorner.getX()),
                backRightCorner.getX());
        double maxY = Math.max(
                Math.max(Math.max(frontLeftCorner.getY(), backLeftCorner.getY()), frontRightCorner.getY()),
                backRightCorner.getY());
        double minY = Math.min(
                Math.min(Math.min(frontLeftCorner.getY(), backLeftCorner.getY()), frontRightCorner.getY()),
                backRightCorner.getY());

        double xVel = vel.getX();
        double yVel = vel.getY();

        if (maxX > FieldConstants.FIELD_LENGTH.plus(DISTANCE_INTO_WALL).in(Meters)) {
            xVel = Math.min(xVel, 0);
        } else if (minX < -DISTANCE_INTO_WALL.in(Meters)) {
            xVel = Math.max(xVel, 0);
        }

        if (maxY > FieldConstants.FIELD_WIDTH.plus(DISTANCE_INTO_WALL).in(Meters)) {
            yVel = Math.min(yVel, 0);
        } else if (minY < -DISTANCE_INTO_WALL.in(Meters)) {
            yVel = Math.max(yVel, 0);
        }

        return new Translation2d(xVel, yVel);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    private enum DriveMode {
        NORMAL,
        TRENCH_LOCK,
        BUMP_LOCK,
        TOWER_LOCK
    }
}
