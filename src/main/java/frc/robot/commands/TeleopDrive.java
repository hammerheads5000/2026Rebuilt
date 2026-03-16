// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.SlewRateLimiter2d;
import frc.robot.util.TunableControls.TunablePIDController;
import frc.robot.util.Zones;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

/** Default drive command to run that drives based on controller input */
public class TeleopDrive extends Command {
    private final Drive drive;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier omegaSupplier;
    private final SlewRateLimiter2d driveLimiter;
    private int flipFactor = 1; // 1 for normal, -1 for flipped

    private LinearVelocity maxDriveSpeed = SwerveConstants.DEFAULT_DRIVE_SPEED;
    private AngularVelocity maxRotSpeed = SwerveConstants.DEFAULT_ROT_SPEED;

    @AutoLogOutput
    private final Trigger inTrenchZoneTrigger;

    @AutoLogOutput
    private final Trigger inBumpZoneTrigger;

    private final TunablePIDController trenchYController =
            new TunablePIDController(SwerveConstants.TRENCH_TRANSLATION_CONSTANTS);
    private final TunablePIDController rotationController =
            new TunablePIDController(SwerveConstants.ROTATION_CONSTANTS);

    @AutoLogOutput
    private DriveMode currentDriveMode = DriveMode.NORMAL;

    /** Creates a new TeleopDrive. */
    public TeleopDrive(Drive drive, CommandXboxController controller) {
        this.drive = drive;
        this.xSupplier = () -> -controller.getLeftY() * flipFactor;
        this.ySupplier = () -> -controller.getLeftX() * flipFactor;
        this.omegaSupplier = () -> -controller.getRightX();
        this.driveLimiter = new SlewRateLimiter2d(SwerveConstants.MAX_TELEOP_ACCEL.in(MetersPerSecondPerSecond));

        inTrenchZoneTrigger = Zones.TRENCH_ZONES
                .willContain(drive::getPose, drive::getFieldSpeeds, SwerveConstants.TRENCH_ALIGN_TIME)
                .debounce(0.1);

        inBumpZoneTrigger = Zones.BUMP_ZONES
                .willContain(drive::getPose, drive::getFieldSpeeds, SwerveConstants.BUMP_ALIGN_TIME)
                .debounce(0.1);

        inTrenchZoneTrigger.onTrue(updateDriveMode(DriveMode.TRENCH_LOCK));
        inBumpZoneTrigger.onTrue(updateDriveMode(DriveMode.BUMP_LOCK));
        inTrenchZoneTrigger.or(inBumpZoneTrigger).onFalse(updateDriveMode(DriveMode.NORMAL));

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

    private Rotation2d getTrenchLockAngle() {
        if (Math.abs(MathUtil.inputModulus(drive.getRotation().getDegrees() - 90, -180, 180)) < 90) {
            return Rotation2d.kCCW_90deg;
        } else {
            return Rotation2d.kCW_90deg;
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

        switch (currentDriveMode) {
            case NORMAL:
                drive.driveFieldCentric(
                        MetersPerSecond.of(linearVelocity.getX()),
                        MetersPerSecond.of(linearVelocity.getY()),
                        maxRotSpeed.times(omega));
                break;
            case TRENCH_LOCK:
                trenchYController.setSetpoint(getTrenchY().in(Meters));
                double yVel = trenchYController.calculate(drive.getPose().getY());
                if (trenchYController.atSetpoint()) {
                    yVel = 0;
                }
                rotationController.setSetpoint(getTrenchLockAngle().getRadians());
                double rotSpeedToStraight =
                        rotationController.calculate(drive.getRotation().getRadians());
                if (rotationController.atSetpoint()) {
                    rotSpeedToStraight = 0;
                }
                drive.driveFieldCentric(
                        MetersPerSecond.of(linearVelocity.getX()),
                        MetersPerSecond.of(yVel),
                        RadiansPerSecond.of(rotSpeedToStraight));
                break;
            case BUMP_LOCK:
                rotationController.setSetpoint(getBumpLockAngle().getRadians());
                double rotSpeedToDiagonal =
                        rotationController.calculate(drive.getRotation().getRadians());
                if (rotationController.atSetpoint()) {
                    rotSpeedToDiagonal = 0;
                }
                drive.driveFieldCentric(
                        MetersPerSecond.of(linearVelocity.getX()),
                        MetersPerSecond.of(linearVelocity.getY()),
                        RadiansPerSecond.of(rotSpeedToDiagonal));
                break;
        }
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
                });
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
                });
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
        BUMP_LOCK
    }
}
