// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.TunableControls.TunableControlConstants;
import frc.robot.util.TunableControls.TunablePIDController;
import org.littletonrobotics.junction.Logger;

/**
 * Command to align the robot to a specific pose (translation + rotation) using PID controllers. It uses profiled PID
 * controllers for translation and a standard PID controller for rotation.
 */
public class AlignToPoseCommand extends Command {
    public final Pose2d targetPose;
    private final TunablePIDController pidControllerX;
    private final TunablePIDController pidControllerY;
    private final TunablePIDController pidControllerAngle;

    private final Drive drive;

    /**
     * Aligns the robot to a given pose (translation + rotation).
     *
     * @param targetPose The target pose to align to.
     * @param linearControlConstants The control constants for linear movement (meters).
     * @param angleControlConstants The control constants for angular movement (degrees).
     * @param drive The drive subsystem.
     */
    public AlignToPoseCommand(
            Pose2d targetPose,
            TunableControlConstants linearControlConstants,
            TunableControlConstants angleControlConstants,
            Drive drive) {
        this.targetPose = targetPose;
        this.drive = drive;

        pidControllerX = new TunablePIDController(linearControlConstants);
        pidControllerY = new TunablePIDController(linearControlConstants);
        pidControllerAngle = new TunablePIDController(angleControlConstants);

        pidControllerX.setSetpoint(0);
        pidControllerY.setSetpoint(0);
        pidControllerAngle.setSetpoint(targetPose.getRotation().getDegrees());

        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Reset PIDControllers to initial position and velocity
        ChassisSpeeds chassisSpeeds = getRelativeSpeeds(drive.getFieldSpeeds());
        pidControllerX.reset();
        pidControllerY.reset();
        pidControllerAngle.reset();

        pidControllerX.setSetpoint(0);
        pidControllerY.setSetpoint(0);
        pidControllerAngle.setSetpoint(targetPose.getRotation().getDegrees());

        Logger.recordOutput("Alignment/Aligned", false);
        Logger.recordOutput("Alignment/TargetPose", targetPose);
    }

    private ChassisSpeeds getRelativeSpeeds(ChassisSpeeds fieldSpeeds) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, targetPose.getRotation());
    }

    private ChassisSpeeds getFieldSpeeds(ChassisSpeeds relativeSpeeds) {
        return ChassisSpeeds.fromRobotRelativeSpeeds(relativeSpeeds, targetPose.getRotation());
    }

    private Pose2d getRelativePose() {
        return drive.getPose().relativeTo(targetPose);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d relativePose = getRelativePose();
        double xVel = pidControllerX.calculate(relativePose.getX());
        double yVel = pidControllerY.calculate(relativePose.getY());
        double omega = DegreesPerSecond.of(
                        pidControllerAngle.calculate(drive.getRotation().getDegrees()))
                .in(RadiansPerSecond);

        // target pose relative -> field relative -> robot relative
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                getFieldSpeeds(new ChassisSpeeds(xVel, yVel, omega)), drive.getRotation());

        drive.runVelocity(speeds);

        Pose2d setpoint = targetPose.transformBy(new Transform2d(
                new Translation2d(pidControllerX.getSetpoint(), pidControllerY.getSetpoint()), Rotation2d.kZero));

        Logger.recordOutput(
                "Alignment/Setpoint (field relative)",
                new Pose2d(setpoint.getX(), setpoint.getY(), Rotation2d.fromDegrees(pidControllerAngle.getSetpoint())));
        Logger.recordOutput("Alignment/setpointX", pidControllerX.getSetpoint());
        Logger.recordOutput("Alignment/setpointY", pidControllerY.getSetpoint());
        Logger.recordOutput("Alignment/measuredX", relativePose.getX());
        Logger.recordOutput("Alignment/measuredY", relativePose.getY());
        Logger.recordOutput("Alignment/Distance to Target", getDistanceToTarget());
    }

    public Distance getDistanceToTarget() {
        return Meters.of(getRelativePose().getTranslation().getNorm());
    }

    public Trigger withinDistanceToTarget(Distance distance) {
        return new Trigger(() -> getDistanceToTarget().lt(distance));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Logger.recordOutput("Alignment/Aligned", !interrupted);
        ;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
