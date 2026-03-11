package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.intake.Intakes;
import frc.robot.subsystems.intake.Intakes.IntakesGoal;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretGoal;
import frc.robot.subsystems.vision.Vision;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import org.json.simple.parser.ParseException;

/** Builds autos from PathPlanner paths selected using SharkPlanner */
public class AutoCreator {
    /** A set waypoint that can be the start or end of a path */
    public enum StartEndPoint {
        TRENCH_START_LEFT("Trench Start Left", MetersPerSecond.of(0)),
        TRENCH_START_RIGHT("Trench Start Right", MetersPerSecond.of(0)),
        TRENCH_MID_START_LEFT("Trench Mid Start Left", MetersPerSecond.of(0)),
        TRENCH_MID_START_RIGHT("Trench Mid Start Right", MetersPerSecond.of(0)),
        TRENCH_RIGHT("Trench Right", AutoConstants.HANDOFF_VELOCITY),
        TRENCH_LEFT("Trench Left", AutoConstants.HANDOFF_VELOCITY),
        CLIMB_RIGHT("Climb Right", MetersPerSecond.of(0)),
        CLIMB_LEFT("Climb Left", MetersPerSecond.of(0)),
        BACK_CLIMB_RIGHT("Back Climb Right", MetersPerSecond.of(0)),
        BACK_CLIMB_LEFT("Back Climb Left", MetersPerSecond.of(0)),
        DUMP_RIGHT("Dump Right", MetersPerSecond.of(0)),
        DUMP_LEFT("Dump Left", MetersPerSecond.of(0)),
        BUMP_START_LEFT("Bump Start Left", MetersPerSecond.of(0)),
        BUMP_START_RIGHT("Bump Start Right", MetersPerSecond.of(0)),
        BUMP_LEFT("Bump Left", AutoConstants.HANDOFF_VELOCITY),
        BUMP_RIGHT("Bump Right", AutoConstants.HANDOFF_VELOCITY),
        BUMP_DUMP_LEFT("Bump Dump Left", MetersPerSecond.of(0)),
        BUMP_DUMP_RIGHT("Bump Dump Right", MetersPerSecond.of(0)),
        DEPOT("Depot", MetersPerSecond.of(0)),
        OUTPOST("Outpost", MetersPerSecond.of(0));

        private String name;
        private LinearVelocity handoffVelocity; // what velocity the ideal start/end state for this point should have

        private StartEndPoint(String name, LinearVelocity handoffVelocity) {
            this.name = name;
            this.handoffVelocity = handoffVelocity;
        }

        private static StartEndPoint fromString(String s) {
            for (StartEndPoint point : values()) {
                if (point.name.equalsIgnoreCase(s)) {
                    return point;
                }
            }
            throw new IllegalArgumentException("Invalid start/end point: " + s);
        }
    }

    /** Wrapper of a PathPlanner path to get useful information */
    public static class AutoPath {
        protected final StartEndPoint start;
        protected final StartEndPoint end;
        protected final String name;
        protected final boolean
                collect; // whether the path should collect instead of passing (only applies to collection paths)
        protected final double dumpTime; // time to spend still at the end of the path to shoot fuel
        private PathPlannerPath pathPlannerPath = null; // for caching
        private PathPlannerTrajectory idealTrajectory = null; // for caching

        private AutoPath(StartEndPoint start, StartEndPoint end, String name, boolean collect, double dumpTime) {
            this.start = start;
            this.end = end;
            this.name = name;
            this.collect = collect;
            this.dumpTime = dumpTime;
        }

        // parse path file name
        protected static AutoPath fromString(String s) {
            if (s.startsWith("Collect")) {
                return fromCollectString(s);
            }

            String[] parts = s.split(" to ");
            if (parts.length != 2) {
                throw new IllegalArgumentException("Invalid auto path: " + s);
            }
            double dumpTime = 0;
            if (parts[1].split("=").length == 2) {
                dumpTime = Double.parseDouble(parts[1].split("=")[1]);
                parts[1] = parts[1].split("=")[0];
                s = parts[0] + " to " + parts[1];
            }
            StartEndPoint start = StartEndPoint.fromString(parts[0]);
            StartEndPoint end = StartEndPoint.fromString(parts[1]);
            return new AutoPath(start, end, s, false, dumpTime);
        }

        /** parse file name for collection paths (automatically called by fromString) */
        protected static AutoPath fromCollectString(String s) {
            boolean collect = false;

            if (s.endsWith("!")) {
                collect = true;
                s = s.substring(0, s.length() - 1);
            }

            StartEndPoint startEnd;
            if (s.contains("Left Bump")) {
                startEnd = StartEndPoint.BUMP_LEFT;
            } else if (s.contains("Right Bump")) {
                startEnd = StartEndPoint.BUMP_RIGHT;
            } else if (s.contains("Left")) {
                startEnd = StartEndPoint.TRENCH_LEFT;
            } else {
                startEnd = StartEndPoint.TRENCH_RIGHT;
            }

            return new AutoPath(startEnd, startEnd, s, collect, 0);
        }

        /** returns {@link PathPlannerPath} represented by this AutoPath */
        protected PathPlannerPath getPathPlannerPath() {
            if (pathPlannerPath != null) {
                return pathPlannerPath;
            }
            try {
                PathPlannerPath path = PathPlannerPath.fromPathFile(name);
                List<ConstraintsZone> constraintZones = path.getConstraintZones();
                PathConstraints constraints = AutoConstants.SCORING_CONSTRAINTS;
                if (name.contains("Collect")) {
                    constraintZones.clear();
                    constraintZones.add(new ConstraintsZone(1, 2, AutoConstants.COLLECT_CONSTRAINTS));
                    constraints = AutoConstants.CONSTRAINTS;
                }

                path = new PathPlannerPath(
                        path.getWaypoints(),
                        path.getRotationTargets(),
                        path.getPointTowardsZones(),
                        constraintZones,
                        path.getEventMarkers(),
                        constraints,
                        new IdealStartingState(
                                start.handoffVelocity,
                                path.getIdealStartingState().rotation()),
                        new GoalEndState(
                                end.handoffVelocity, path.getGoalEndState().rotation()),
                        false);
                this.pathPlannerPath = path;
                return path;
            } catch (FileVersionException | IOException | ParseException e) {
                throw new RuntimeException("Failed to load path: " + name, e);
            }
        }

        protected List<PathPlannerTrajectoryState> getTrajectoryStates() {
            if (idealTrajectory == null) {
                idealTrajectory = getPathPlannerPath()
                        .getIdealTrajectory(AutoConstants.PP_CONFIG)
                        .orElse(new PathPlannerTrajectory(new ArrayList<>()));
            }
            ArrayList<PathPlannerTrajectoryState> states = new ArrayList<>(idealTrajectory.getStates());
            if (dumpTime > 0) {
                PathPlannerTrajectoryState lastState = states.get(states.size() - 1);
                states.add(lastState.copyWithTime(lastState.timeSeconds + dumpTime));
            }

            return states;
        }

        /** get the total time it would take to complete the ideal trajectory */
        protected double getTotalTime() {
            if (idealTrajectory == null) {
                idealTrajectory = getPathPlannerPath()
                        .getIdealTrajectory(AutoConstants.PP_CONFIG)
                        .orElse(new PathPlannerTrajectory(new ArrayList<>()));
            }
            return idealTrajectory.getTotalTimeSeconds() + dumpTime;
        }
    }

    private EnumMap<StartEndPoint, List<AutoPath>> autoPathsByStartPoint =
            new EnumMap<>(StartEndPoint.class); // determines which start points can start which paths
    private ArrayList<AutoPath> selectedAutoPaths = new ArrayList<>(); // paths selected by SharkPlanner
    private String lastAutoSelection = ""; // cached auto selection to determine value change
    private ArrayList<PathPlannerTrajectoryState> trajStates = new ArrayList<>();
    private final StringArrayPublisher autoOptionsPub = AutoConstants.AUTO_OPTIONS.publish(); // available auto paths
    private final DoubleArrayPublisher autoOptionTimesPub =
            AutoConstants.AUTO_OPTION_TIMES.publish(); // times corresponding to each available path
    private final StringArrayPublisher startOptionsPub =
            AutoConstants.START_OPTIONS.publish(); // available start points
    private final StructArrayPublisher<Translation2d> trajPub = AutoConstants.TRAJECTORY.publish();
    private final DoubleArrayPublisher trajTimePub = AutoConstants.TRAJECTORY_TIMESTAMPS.publish();
    private final DoublePublisher timestampPub = AutoConstants.TIMESTAMP.publish(); // current timestamp

    /**
     * Load PathPlannerPaths from the rio's deploy directory
     * @throws RuntimeException when the paths fail to load
     */
    public void loadPathplannerPaths() {
        File pathplannerDir = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");
        for (String s : pathplannerDir.list()) System.out.println(s);
        if (!pathplannerDir.exists()) {
            throw new RuntimeException("Pathplanner directory does not exist");
        }

        File[] pathFiles = pathplannerDir.listFiles((dir, name) -> name.endsWith(".path"));
        if (pathFiles == null) {
            throw new RuntimeException("Failed to list path files");
        }

        ArrayList<AutoPath> autoPaths = new ArrayList<>(pathFiles.length);

        for (File pathFile : pathFiles) {
            String fileName = pathFile.getName();
            String nameWithoutExtension = fileName.substring(0, fileName.length() - 5);
            autoPaths.add(AutoPath.fromString(nameWithoutExtension));
        }

        for (StartEndPoint point : StartEndPoint.values()) {
            autoPathsByStartPoint.put(
                    point,
                    autoPaths.stream().filter(path -> path.start == point).toList());
        }
    }

    /** Builds autos from PathPlanner paths selected using SharkPlanner */
    public AutoCreator() {
        loadPathplannerPaths();
        autoOptionsPub.set(new String[0]);
        autoOptionTimesPub.set(new double[0]);

        startOptionsPub.set(new String[] {
            StartEndPoint.TRENCH_START_LEFT.name,
            StartEndPoint.TRENCH_START_RIGHT.name,
            StartEndPoint.TRENCH_MID_START_LEFT.name,
            StartEndPoint.TRENCH_MID_START_RIGHT.name,
            StartEndPoint.BUMP_START_LEFT.name,
            StartEndPoint.BUMP_START_RIGHT.name
        });
        timestampPub.set(Timer.getFPGATimestamp());
    }

    private static String[] autoPathsToStringArray(List<AutoPath> autoPaths) {
        String[] arr = new String[autoPaths.size()];
        for (int i = 0; i < arr.length; i++) {
            arr[i] = autoPaths.get(i).name;
        }

        return arr;
    }

    public static ArrayList<AutoPath> autoPathsFromString(String selected) {
        String[] parts = selected.split(";");
        if (parts.length == 1 && parts[0].isEmpty()) {
            return new ArrayList<>();
        }

        // only start point
        if (parts.length == 1) {
            return new ArrayList<>();
        }

        // update auto paths
        ArrayList<AutoPath> autoPaths = new ArrayList<>();
        for (int i = 1; i < parts.length; i++) {
            autoPaths.add(AutoPath.fromString(parts[i].trim()));
        }

        return autoPaths;
    }

    /** updates subscribers and publishes values to NetworkTables, recalculating auto paths if necessary */
    public void updateNT() {
        String selected = AutoConstants.AUTO_SELECTION.get();
        if (selected == lastAutoSelection) {
            return;
        }

        String[] parts = selected.split(";");
        if (parts.length == 1 && parts[0].isEmpty()) {
            selectedAutoPaths.clear();
            return;
        }

        try {
            StartEndPoint selectedStart = StartEndPoint.fromString(parts[0].trim());

            // only start point
            if (parts.length == 1) {
                List<AutoPath> autoPaths = autoPathsByStartPoint.get(selectedStart);
                autoOptionsPub.set(autoPathsToStringArray(autoPaths));
                autoOptionTimesPub.set(autoPaths.stream()
                        .mapToDouble((stream) -> stream.getTotalTime())
                        .toArray());
                timestampPub.set(Timer.getFPGATimestamp());
                selectedAutoPaths.clear();
                return;
            }

            // update auto paths
            selectedAutoPaths.clear();
            for (int i = 1; i < parts.length; i++) {
                selectedAutoPaths.add(AutoPath.fromString(parts[i].trim()));
            }
        } catch (IllegalArgumentException e) {
            System.out.println(e.getMessage()); // thrown when StartEndPoint doesn't exist
            return;
        }

        try {
            updateCurrentTrajectoryStates();
        } catch (RuntimeException e) {
            System.out.println("Trajectory state update error: " + e.getMessage());
            return;
        }

        // update NT values
        List<AutoPath> autoPaths = autoPathsByStartPoint.get(selectedAutoPaths.get(selectedAutoPaths.size() - 1).end);
        autoOptionsPub.set(autoPathsToStringArray(autoPaths));
        autoOptionTimesPub.set(autoPaths.stream()
                .mapToDouble((stream) -> stream.getTotalTime())
                .toArray());
        trajPub.set(getCurrentTrajectory());
        trajTimePub.set(getCurrentTrajectoryTimestamps());
        timestampPub.set(Timer.getFPGATimestamp());
    }

    private void updateCurrentTrajectoryStates() {
        trajStates.clear();
        for (AutoPath path : selectedAutoPaths) {
            trajStates.addAll(path.getTrajectoryStates());
        }
    }

    public Translation2d[] getCurrentTrajectory() {
        return trajStates.stream().map((state) -> state.pose.getTranslation()).toArray(Translation2d[]::new);
    }

    private double[] getCurrentTrajectoryTimestamps() {
        double[] timestamps = new double[trajStates.size()];
        double totalTime = 0;
        timestamps[0] = 0;
        for (int i = 1; i < timestamps.length; i++) {
            if (trajStates.get(i).timeSeconds < trajStates.get(i - 1).timeSeconds) {
                totalTime += trajStates.get(i - 1).timeSeconds;
            }
            timestamps[i] = trajStates.get(i).timeSeconds + totalTime;
        }
        return timestamps;
    }

    /**
     * Builds auto from the currently selected paths in SharkPlanner
     */
    public Command buildAuto(
            Drive drive,
            Vision vision,
            Intakes intakes,
            Indexer indexer,
            Turret turret,
            Climber climber,
            Superstructure superstructure) {
        return AutoCreator.buildAuto(
                drive,
                vision,
                intakes,
                indexer,
                turret,
                climber,
                superstructure,
                selectedAutoPaths,
                AutoConstants.DUMP_AT_START.get());
    }

    /**
     * Builds auto from the given paths
     */
    public static Command buildAuto(
            Drive drive,
            Vision vision,
            Intakes intakes,
            Indexer indexer,
            Turret turret,
            Climber climber,
            Superstructure superstructure,
            List<AutoPath> selectedAutoPaths,
            boolean dumpAtStart) {
        ArrayList<Command> commands = new ArrayList<>();
        for (AutoPath path : selectedAutoPaths) {
            Command toAdd = AutoBuilder.followPath(path.getPathPlannerPath());
            if (path.name.contains("Collect")) {
                if (path.collect) {
                    toAdd = toAdd.deadlineFor(
                            indexer.setGoal(IndexerGoal.IDLE),
                            turret.setGoal(TurretGoal.IDLE).asProxy());
                } else {
                    toAdd = toAdd.deadlineFor(
                            indexer.setGoal(IndexerGoal.ACTIVE),
                            turret.setGoal(TurretGoal.PASSING).asProxy());
                }

                // deploy appropriate intake
                if (path.start == StartEndPoint.TRENCH_LEFT) {
                    toAdd = toAdd.deadlineFor(intakes.deployLeft());
                } else if (path.start == StartEndPoint.TRENCH_RIGHT) {
                    toAdd = toAdd.deadlineFor(intakes.deployRight());
                } else if (path.start == StartEndPoint.BUMP_LEFT) {
                    toAdd = toAdd.deadlineFor(intakes.deployLeft());
                } else if (path.start == StartEndPoint.BUMP_RIGHT) {
                    toAdd = toAdd.deadlineFor(intakes.deployRight());
                }
            } else if (!((path.start == StartEndPoint.TRENCH_START_LEFT && path.end == StartEndPoint.TRENCH_LEFT)
                    || (path.start == StartEndPoint.TRENCH_START_RIGHT && path.end == StartEndPoint.TRENCH_RIGHT)
                    || (path.start == StartEndPoint.TRENCH_MID_START_LEFT && path.end == StartEndPoint.TRENCH_LEFT)
                    || (path.start == StartEndPoint.TRENCH_MID_START_RIGHT && path.end == StartEndPoint.TRENCH_RIGHT)
                    || (path.start == StartEndPoint.BUMP_START_LEFT && path.end == StartEndPoint.BUMP_LEFT)
                    || (path.start == StartEndPoint.BUMP_START_RIGHT
                            && path.end == StartEndPoint.BUMP_RIGHT))) { // if not passing through trench or over bump
                // set turret to score when possible
                toAdd = toAdd.deadlineFor(Commands.waitUntil(superstructure.inAllianceZoneTrigger)
                        .andThen(turret.setGoal(TurretGoal.SCORING)
                                .asProxy()
                                .alongWith(Commands.waitTime(AutoConstants.START_SPIN_UP_TIME)
                                        .andThen(indexer.setGoal(IndexerGoal.ACTIVE)))));
            }

            if (path.dumpTime > 0) {
                toAdd = toAdd.andThen(Commands.waitSeconds(path.dumpTime));
            }

            // add climbing to the end if applicable
            if (path.end == StartEndPoint.CLIMB_LEFT || path.end == StartEndPoint.CLIMB_RIGHT) {
                toAdd = toAdd.alongWith(climber.extend(), intakes.setGoal(IntakesGoal.STOW))
                        .andThen(
                                Commands.waitUntil(() ->
                                        DriverStation.getMatchTime() <= AutoConstants.CLIMB_TIME_REMAINING.in(Seconds)),
                                AutoClimb.getAutoClimbCommand(drive, vision, climber));
            }

            // wait for alignment to extend climber
            if (path.end == StartEndPoint.BACK_CLIMB_LEFT || path.end == StartEndPoint.BACK_CLIMB_RIGHT) {
                toAdd = toAdd.alongWith(intakes.setGoal(IntakesGoal.STOW))
                        .andThen(
                                Commands.waitUntil(() ->
                                        DriverStation.getMatchTime() <= AutoConstants.CLIMB_TIME_REMAINING.in(Seconds)),
                                AutoClimb.getAutoClimbCommand(drive, vision, climber));
            }

            if (path.end == StartEndPoint.DEPOT) {
                toAdd = toAdd.alongWith(intakes.deployRight());
            }

            if (path.end == StartEndPoint.OUTPOST) {
                toAdd = toAdd.alongWith(intakes.deployLeft());
            }

            commands.add(toAdd);
        }

        // add dump at start if necessary
        if (dumpAtStart) {
            commands.add(
                    0,
                    Commands.parallel(
                            turret.setGoal(TurretGoal.SCORING).asProxy(),
                            Commands.waitTime(AutoConstants.START_SPIN_UP_TIME)
                                    .andThen(indexer.setGoal(IndexerGoal.ACTIVE)),
                            Commands.waitTime(AutoConstants.START_SPIN_UP_TIME.plus(AutoConstants.START_DUMP_TIME))));
        } else {
            commands.add(
                    0,
                    indexer.setGoal(IndexerGoal.IDLE)
                            .alongWith(turret.setGoal(TurretGoal.IDLE).asProxy()));
        }

        // turn commands ArrayList into a sequential command group
        return Commands.sequence(commands.toArray(Command[]::new)).withName("Created Auto");
    }
}
