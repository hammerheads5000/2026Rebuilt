// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.LoggedTunableNumber;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
    protected final PhotonCamera camera;

    protected Transform3d robotToCamera;
    LoggedTunableNumber robotToCamRoll;
    LoggedTunableNumber robotToCamPitch;
    LoggedTunableNumber robotToCamYaw;

    /**
     * Creates a new VisionIOPhotonVision.
     *
     * @param name The configured name of the camera.
     * @param robotToCamera The 3D position of the camera relative to the robot.
     */
    public VisionIOPhotonVision(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
        robotToCamRoll = new LoggedTunableNumber(
                name + "/Roll", robotToCamera.getRotation().getX());
        robotToCamPitch = new LoggedTunableNumber(
                name + "/Pitch", robotToCamera.getRotation().getY());
        robotToCamYaw = new LoggedTunableNumber(
                name + "/Yaw", robotToCamera.getRotation().getZ());
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        robotToCamera = new Transform3d(
                robotToCamera.getTranslation(),
                new Rotation3d(robotToCamRoll.get(), robotToCamPitch.get(), robotToCamYaw.get()));

        // Read new camera observations
        Set<Short> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        results = results.subList(0, Math.min(5, results.size())); // Limit to 5 results to prevent overload
        for (var result : results) {
            // Update latest target observation
            if (result.hasTargets()) {
                inputs.latestTargetObservation = new TargetObservation(
                        Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                        Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
            } else {
                inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
                continue;
            }

            if (result.getBestTarget().getPoseAmbiguity() > MAX_AMBIGUITY) {
                continue; // Skip targets with high ambiguity
            }

            // Add pose observation
            if (result.multitagResult.isPresent()) { // Multitag result
                var multitagResult = result.multitagResult.get();

                // Calculate robot pose
                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                // Calculate average tag distance
                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance +=
                            target.bestCameraToTarget.getTranslation().getNorm();
                }

                // Add tag IDs
                tagIds.addAll(multitagResult.fiducialIDsUsed);

                // Add observation
                poseObservations.add(new PoseObservation(
                        result.getTimestampSeconds(), // Timestamp
                        robotPose, // 3D pose estimate
                        multitagResult.estimatedPose.ambiguity, // Ambiguity
                        multitagResult.fiducialIDsUsed.size(), // Tag count
                        multitagResult.fiducialIDsUsed.stream()
                                .anyMatch(id -> (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                                ? BLUE_HUB_TAG_IDS
                                                : RED_HUB_TAG_IDS)
                                        .contains(id.intValue())), // Has hub tag
                        totalTagDistance / result.targets.size(), // Average tag distance
                        PoseObservationType.PHOTONVISION)); // Observation type

            } else if (!result.targets.isEmpty()) { // Single tag result
                var target = result.targets.get(0);

                // Calculate robot pose
                var tagPose = APRIL_TAGS.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(
                            tagPose.get().getTranslation(), tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    // Add tag ID
                    tagIds.add((short) target.fiducialId);

                    // Add observation
                    poseObservations.add(new PoseObservation(
                            result.getTimestampSeconds(), // Timestamp
                            robotPose, // 3D pose estimate
                            target.poseAmbiguity, // Ambiguity
                            1, // Tag count
                            (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
                                            ? BLUE_HUB_TAG_IDS
                                            : RED_HUB_TAG_IDS)
                                    .contains(target.fiducialId), // Has hub tag
                            cameraToTarget.getTranslation().getNorm(), // Average tag distance
                            PoseObservationType.PHOTONVISION)); // Observation type
                }
            }
        }

        // Save pose observations to inputs object
        inputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
            inputs.poseObservations[i] = poseObservations.get(i);
        }

        // Save tag IDs to inputs objects
        inputs.tagIds = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) {
            inputs.tagIds[i++] = id;
        }
    }

    @Override
    public String getName() {
        return camera.getName();
    }
}
