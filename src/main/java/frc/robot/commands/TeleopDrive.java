// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.SlewRateLimiter2d;
import java.util.function.DoubleSupplier;

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

    /** Creates a new TeleopDrive. */
    public TeleopDrive(Drive drive, CommandXboxController controller) {
        this.drive = drive;
        this.xSupplier = () -> -controller.getLeftY() * flipFactor;
        this.ySupplier = () -> -controller.getLeftX() * flipFactor;
        this.omegaSupplier = () -> -controller.getRightX();
        this.driveLimiter = new SlewRateLimiter2d(SwerveConstants.MAX_TELEOP_ACCEL.in(MetersPerSecondPerSecond));

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

        drive.driveFieldCentric(
                MetersPerSecond.of(linearVelocity.getX()),
                MetersPerSecond.of(linearVelocity.getY()),
                maxRotSpeed.times(omega));
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

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
