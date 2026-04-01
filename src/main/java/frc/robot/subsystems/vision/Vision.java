// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    private final VisionConsumer consumer;
    private final Consumer<Pose2d> resetConsumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;
    private final Alert disabledAlert = new Alert("Vision Disabled", AlertType.kWarning);

    private boolean disabled = false;
    private boolean climbing = false;

    @AutoLogOutput
    private boolean hasHadVision = false;

    public Vision(VisionConsumer consumer, Consumer<Pose2d> resetConsumer, VisionIO... io) {
        this.consumer = consumer;
        this.resetConsumer = resetConsumer;
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] =
                    new Alert("Vision camera " + io[i].getName() + " is disconnected.", AlertType.kWarning);
        }

        SmartDashboard.putData("Overrides/Vision", disable());
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing with vision.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + io[i].getName(), inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        boolean isBlue = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
        Set<Integer> hubTags = isBlue ? BLUE_HUB_TAG_IDS : RED_HUB_TAG_IDS;
        Set<Integer> climbTags = isBlue ? BLUE_CLIMB_TAG_IDS : RED_CLIMB_TAG_IDS;

        // Loop over cameras
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update disconnected alert
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize logging values
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Loop over pose observations
            for (var observation : inputs[cameraIndex].poseObservations) {
                // Check whether to reject pose
                boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag
                        || (observation.tagCount() == 1
                                && observation.ambiguity() > MAX_AMBIGUITY) // Cannot be high ambiguity
                        || observation.pose().getZ() > MAX_Z_HEIGHT.in(Meters)
                        // Must be within the field boundaries
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > APRIL_TAGS.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > APRIL_TAGS.getFieldWidth();

                // Add pose to log
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip if rejected
                if (rejectPose) {
                    continue;
                }

                // Calculate standard deviations
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();

                // Add tag poses
                for (int tagId : inputs[cameraIndex].tagIds) {
                    var tagPose = APRIL_TAGS.getTagPose(tagId);
                    if (tagPose.isPresent()) {
                        tagPoses.add(tagPose.get());
                    }
                    if (climbing && !climbTags.contains(tagId)) {
                        stdDevFactor += CLIMB_TAG_STD_DEV_BIAS; // Non-climb tags are less accurate
                    } else if (!climbing && !hubTags.contains(tagId)) {
                        stdDevFactor += HUB_TAG_STD_DEV_BIAS; // Non-hub tags are less accurate
                    }
                }
                double linearStdDev = LINEAR_STD_DEV_BASELINES[cameraIndex] * stdDevFactor;
                double angularStdDev = ANGULAR_STD_DEV_BASELINE * stdDevFactor;

                // Send vision observation
                if (!disabled) {
                    if (hasHadVision) {
                        consumer.accept(
                                observation.pose().toPose2d(),
                                observation.timestamp(),
                                VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
                    } else {
                        resetConsumer.accept(observation.pose().toPose2d());
                        hasHadVision = true;
                    }
                }
            }

            // Log camera metadata
            Logger.recordOutput("Vision/" + io[cameraIndex].getName() + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
            Logger.recordOutput(
                    "Vision/" + io[cameraIndex].getName() + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
            Logger.recordOutput(
                    "Vision/" + io[cameraIndex].getName() + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[0]));
            Logger.recordOutput(
                    "Vision/" + io[cameraIndex].getName() + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[0]));
            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log summary data
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
    }

    public Command disable() {
        return this.runOnce(() -> {
                    disabled = true;
                    disabledAlert.set(true);
                })
                .andThen(Commands.idle())
                .finallyDo(() -> {
                    disabled = false;
                    disabledAlert.set(false);
                })
                .ignoringDisable(true)
                .withName("Disable Vision");
    }

    public void setClimbing(boolean climbing) {
        this.climbing = climbing;
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void accept(
                Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs);
    }
}
