package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class FuelSim {
    private static final double PERIOD = 0.02; // sec
    private static int subticks = 5;
    private static final Translation3d GRAVITY = new Translation3d(0, 0, -9.81); // m/s^2
    private static final double FIELD_COR = Math.sqrt(22 / 51.5); // coefficient of restitution with the field
    private static final double FUEL_COR = 0.5; // coefficient of restitution with another fuel
    private static final double ROBOT_COR = 0.3; // coefficient of restitution with a robot
    private static final double FUEL_RADIUS = 0.075;
    private static final double FIELD_LENGTH = 16.51;
    private static final double FIELD_WIDTH = 8.04;
    private static final double FRICTION = 0.1; // proportion of horizontal velocity to lose per second while on ground

    private static FuelSim instance = null;

    private static final Translation3d[] FIELD_XZ_LINE_STARTS = {
        new Translation3d(0, 0, 0),
        new Translation3d(3.96, 1.57, 0),
        new Translation3d(3.96, FIELD_WIDTH / 2 + 0.60, 0),
        new Translation3d(4.61, 1.57, 0.165),
        new Translation3d(4.61, FIELD_WIDTH / 2 + 0.60, 0.165),
        new Translation3d(FIELD_LENGTH - 5.18, 1.57, 0),
        new Translation3d(FIELD_LENGTH - 5.18, FIELD_WIDTH / 2 + 0.60, 0),
        new Translation3d(FIELD_LENGTH - 4.61, 1.57, 0.165),
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 + 0.60, 0.165)
    };

    private static final Translation3d[] FIELD_XZ_LINE_ENDS = {
        new Translation3d(FIELD_LENGTH, FIELD_WIDTH, 0),
        new Translation3d(4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
        new Translation3d(4.61, FIELD_WIDTH - 1.57, 0.165),
        new Translation3d(5.18, FIELD_WIDTH / 2 - 0.60, 0),
        new Translation3d(5.18, FIELD_WIDTH - 1.57, 0),
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2 - 0.60, 0.165),
        new Translation3d(FIELD_LENGTH - 4.61, FIELD_WIDTH - 1.57, 0.165),
        new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH / 2 - 0.60, 0),
        new Translation3d(FIELD_LENGTH - 3.96, FIELD_WIDTH - 1.57, 0)
    };

    private class Fuel {
        private Translation3d pos;
        private Translation3d vel;

        private Fuel(Translation3d pos, Translation3d vel) {
            this.pos = pos;
            this.vel = vel;
        }

        private Fuel(Translation3d pos) {
            this(pos, new Translation3d());
        }

        public void update() {
            pos = pos.plus(vel.times(PERIOD / subticks));
            if (pos.getZ() > FUEL_RADIUS) {
                vel = vel.plus(GRAVITY.times(PERIOD / subticks));
            }
            if (Math.abs(vel.getZ()) < 0.05 && pos.getZ() <= FUEL_RADIUS + 0.03) {
                vel = new Translation3d(vel.getX(), vel.getY(), 0);
                vel = vel.times(1 - FRICTION * PERIOD / subticks);
                // pos = new Translation3d(pos.getX(), pos.getY(), FUEL_RADIUS);
            }
            handleFieldCollisions();
        }

        public void handleXZLineCollision(Translation3d lineStart, Translation3d lineEnd) {
            if (pos.getY() < lineStart.getY() || pos.getY() > lineEnd.getY()) return; // not within y range
            // Convert into 2D
            Translation2d start2d = new Translation2d(lineStart.getX(), lineStart.getZ());
            Translation2d end2d = new Translation2d(lineEnd.getX(), lineEnd.getZ());
            Translation2d pos2d = new Translation2d(pos.getX(), pos.getZ());
            Translation2d lineVec = end2d.minus(start2d);

            // Get closest point on line
            Translation2d projected =
                    start2d.plus(lineVec.times(pos2d.minus(start2d).dot(lineVec) / lineVec.getSquaredNorm()));

            if (projected.getDistance(start2d) + projected.getDistance(end2d) > lineVec.getNorm())
                return; // projected point not on line
            double dist = pos2d.getDistance(projected);
            if (dist > FUEL_RADIUS) return; // not intersecting line
            // Back into 3D
            Translation3d normal = new Translation3d(-lineVec.getY(), 0, lineVec.getX()).div(lineVec.getNorm());

            // Apply collision response
            pos = pos.plus(normal.times(FUEL_RADIUS - dist));
            if (vel.dot(normal) > 0) return; // already moving away from line
            vel = vel.minus(normal.times((1 + FIELD_COR) * vel.dot(normal)));
        }

        public void handleXYRobotLineCollision(Translation3d lineStart, Translation3d lineEnd, Translation2d addVel) {
            if (pos.getZ() < lineStart.getZ() || pos.getZ() > lineEnd.getZ()) return; // not within z range
            // Convert into 2D
            Translation2d start2d = new Translation2d(lineStart.getX(), lineStart.getY());
            Translation2d end2d = new Translation2d(lineEnd.getX(), lineEnd.getY());
            Translation2d pos2d = new Translation2d(pos.getX(), pos.getY());
            Translation2d lineVec = end2d.minus(start2d);

            // Get closest point on line
            Translation2d projected =
                    start2d.plus(lineVec.times(pos2d.minus(start2d).dot(lineVec) / lineVec.getSquaredNorm()));

            if (projected.getDistance(start2d) + projected.getDistance(end2d) > lineVec.getNorm())
                return; // projected point not on line
            double dist = pos2d.getDistance(projected);
            if (dist > FUEL_RADIUS) return; // not intersecting line
            // Back into 3D
            Translation3d normal = new Translation3d(-lineVec.getY(), lineVec.getX(), 0).div(lineVec.getNorm());

            // Apply collision response
            pos = pos.plus(normal.times(FUEL_RADIUS - dist));
            if (vel.dot(normal) <= 0) {
                vel = vel.minus(normal.times((1 + ROBOT_COR) * vel.dot(normal)));
            }

            vel = vel.plus(normal.times(addVel.dot(normal.toTranslation2d())));
        }

        public void handleXYLineCollision(Translation3d lineStart, Translation3d lineEnd) {
            handleXYRobotLineCollision(lineStart, lineEnd, new Translation2d());
        }

        private void handleFieldCollisions() {
            // floor and bumps
            for (int i = 0; i < FIELD_XZ_LINE_STARTS.length; i++) {
                handleXZLineCollision(FIELD_XZ_LINE_STARTS[i], FIELD_XZ_LINE_ENDS[i]);
            }

            // edges
            if (pos.getX() < FUEL_RADIUS && vel.getX() < 0) {
                pos = pos.plus(new Translation3d(FUEL_RADIUS - pos.getX(), 0, 0));
                vel = vel.plus(new Translation3d(-(1 + FIELD_COR) * vel.getX(), 0, 0));
            } else if (pos.getX() > FIELD_LENGTH - FUEL_RADIUS && vel.getX() > 0) {
                pos = pos.plus(new Translation3d(FIELD_LENGTH - FUEL_RADIUS - pos.getX(), 0, 0));
                vel = vel.plus(new Translation3d(-(1 + FIELD_COR) * vel.getX(), 0, 0));
            }

            if (pos.getY() < FUEL_RADIUS && vel.getY() < 0) {
                pos = pos.plus(new Translation3d(0, FUEL_RADIUS - pos.getY(), 0));
                vel = vel.plus(new Translation3d(0, -(1 + FIELD_COR) * vel.getY(), 0));
            } else if (pos.getY() > FIELD_WIDTH - FUEL_RADIUS && vel.getY() > 0) {
                pos = pos.plus(new Translation3d(0, FIELD_WIDTH - FUEL_RADIUS - pos.getY(), 0));
                vel = vel.plus(new Translation3d(0, -(1 + FIELD_COR) * vel.getY(), 0));
            }

            // hubs
            handleHubCollisions(Hub.BLUE_HUB);
            handleHubCollisions(Hub.RED_HUB);
        }

        private void handleHubCollisions(Hub hub) {
            hub.handleHubInteraction(this);
            Translation2d collision = hub.fuelCollideSide(this);
            if (collision.getX() != 0) {
                pos = pos.plus(new Translation3d(collision));
                vel = vel.plus(new Translation3d(-(1 + FIELD_COR) * vel.getX(), 0, 0));
            } else if (collision.getY() != 0) {
                pos = pos.plus(new Translation3d(collision));
                vel = vel.plus(new Translation3d(0, -(1 + FIELD_COR) * vel.getY(), 0));
            }
        }

        public void addImpulse(Translation3d impulse) {
            vel = vel.plus(impulse);
        }
    }

    private static void handleFuelCollision(Fuel a, Fuel b) {
        Translation3d normal = a.pos.minus(b.pos);
        double distance = normal.getNorm();
        if (distance == 0) {
            normal = new Translation3d(1, 0, 0);
            distance = 1;
        }
        normal = normal.div(distance);
        double impulse = 0.5 * (1 + FUEL_COR) * (b.vel.minus(a.vel).dot(normal));
        double intersection = FUEL_RADIUS * 2 - distance;
        a.pos = a.pos.plus(normal.times(intersection / 2));
        b.pos = b.pos.minus(normal.times(intersection / 2));
        a.addImpulse(normal.times(impulse));
        b.addImpulse(normal.times(-impulse));
    }

    private static void handleFuelCollisions(ArrayList<Fuel> fuels) {
        for (int i = 0; i < fuels.size() - 1; i++) {
            for (int j = i + 1; j < fuels.size(); j++) {
                if (fuels.get(i).pos.getDistance(fuels.get(j).pos) < FUEL_RADIUS * 2) {
                    handleFuelCollision(fuels.get(i), fuels.get(j));
                }
            }
        }
    }

    private ArrayList<Fuel> fuels = new ArrayList<Fuel>();
    private boolean running = false;
    private Supplier<Pose2d> robotSupplier = null;
    private Supplier<ChassisSpeeds> robotSpeedsSupplier = null;
    private double robotWidth; // size along the robot's y axis
    private double robotLength; // size along the robot's x axis
    private double bumperHeight;
    private ArrayList<SimIntake> intakes = new ArrayList<>();

    public static FuelSim getInstance() {
        if (instance == null) {
            instance = new FuelSim();
        }

        return instance;
    }

    public void clearFuel() {
        fuels.clear();
    }

    public void spawnStartingFuel() {
        // Center fuel
        Translation3d center = new Translation3d(FIELD_LENGTH / 2, FIELD_WIDTH / 2, FUEL_RADIUS);
        for (int i = 0; i < 15; i++) {
            for (int j = 0; j < 6; j++) {
                fuels.add(new Fuel(center.plus(new Translation3d(0.076 + 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0))));
                fuels.add(new Fuel(center.plus(new Translation3d(-0.076 - 0.152 * j, 0.0254 + 0.076 + 0.152 * i, 0))));
                fuels.add(new Fuel(center.plus(new Translation3d(0.076 + 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0))));
                fuels.add(new Fuel(center.plus(new Translation3d(-0.076 - 0.152 * j, -0.0254 - 0.076 - 0.152 * i, 0))));
            }
        }

        // Depots
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                fuels.add(new Fuel(new Translation3d(0.076 + 0.152 * j, 5.95 + 0.076 + 0.152 * i, FUEL_RADIUS)));
                fuels.add(new Fuel(new Translation3d(0.076 + 0.152 * j, 5.95 - 0.076 - 0.152 * i, FUEL_RADIUS)));
                fuels.add(new Fuel(
                        new Translation3d(FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 + 0.076 + 0.152 * i, FUEL_RADIUS)));
                fuels.add(new Fuel(
                        new Translation3d(FIELD_LENGTH - 0.076 - 0.152 * j, 2.09 - 0.076 - 0.152 * i, FUEL_RADIUS)));
            }
        }

        fuels.add(new Fuel(new Translation3d(4.2, 2, 2), new Translation3d(0, 0, 0)));

        for (int i = 0; i < FIELD_XZ_LINE_STARTS.length; i++) {
            Logger.recordOutput(
                    "Fuel Simulation/Lines/" + i, new Translation3d[] {FIELD_XZ_LINE_STARTS[i], FIELD_XZ_LINE_ENDS[i]});
        }
    }

    public void logFuels() {
        Logger.recordOutput(
                "Fuel Simulation/Fuels", fuels.stream().map((fuel) -> fuel.pos).toArray(Translation3d[]::new));
    }

    public void start() {
        running = true;
    }

    public void stop() {
        running = false;
    }

    public void setSubticks(int subticks) {
        FuelSim.subticks = subticks;
    }

    public void registerRobot(
            double width,
            double length,
            double bumperHeight,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.robotSupplier = poseSupplier;
        this.robotSpeedsSupplier = fieldSpeedsSupplier;
        this.robotWidth = width;
        this.robotLength = length;
        this.bumperHeight = bumperHeight;
    }

    /**
     * To be called periodically
     * Will do nothing if sim is not running
     */
    public void updateSim() {
        if (!running) return;

        stepSim();
    }

    /**
     * Run the simulation forward 1 time step (0.02s)
     */
    public void stepSim() {
        for (int i = 0; i < subticks; i++) {
            for (Fuel fuel : fuels) {
                fuel.update();
            }

            handleFuelCollisions(fuels);

            if (robotSupplier != null) {
                handleRobotCollisions(fuels);
                handleIntakes(fuels);
            }
        }

        logFuels();
    }

    public void spawnFuel(Translation3d pos, Translation3d vel) {
        fuels.add(new Fuel(pos, vel));
    }

    private Translation3d[][] getRobotBumperLines(Pose2d robot) {
        Translation2d[] points = new Translation2d[] {
            new Translation2d(robot.getX() - robotLength / 2, robot.getY() + robotWidth / 2),
            new Translation2d(robot.getX() + robotLength / 2, robot.getY() + robotWidth / 2),
            new Translation2d(robot.getX() - robotLength / 2, robot.getY() - robotWidth / 2),
            new Translation2d(robot.getX() + robotLength / 2, robot.getY() - robotWidth / 2)
        };

        for (int i = 0; i < 4; i++) {
            points[i] = points[i].rotateAround(robot.getTranslation(), robot.getRotation());
        }

        return new Translation3d[][] {
            new Translation3d[] {
                new Translation3d(points[0].getX(), points[0].getY(), 0),
                new Translation3d(points[1].getX(), points[1].getY(), bumperHeight)
            },
            new Translation3d[] {
                new Translation3d(points[1].getX(), points[1].getY(), 0),
                new Translation3d(points[2].getX(), points[2].getY(), bumperHeight)
            },
            new Translation3d[] {
                new Translation3d(points[2].getX(), points[2].getY(), 0),
                new Translation3d(points[3].getX(), points[3].getY(), bumperHeight)
            },
            new Translation3d[] {
                new Translation3d(points[3].getX(), points[3].getY(), 0),
                new Translation3d(points[0].getX(), points[0].getY(), bumperHeight)
            }
        };
    }

    private void handleRobotCollision(Fuel fuel, Pose2d robot, Translation2d robotVel, Translation3d[][] bumperLines) {
        Translation2d relativePos = new Pose2d(fuel.pos.toTranslation2d(), Rotation2d.kZero)
                .relativeTo(robot)
                .getTranslation();

        if (fuel.pos.getZ() > bumperHeight - 0.1) return; // above bumpers
        double distanceToLeft = -FUEL_RADIUS - robotLength / 2 - relativePos.getX();
        double distanceToRight = -FUEL_RADIUS - robotLength / 2 + relativePos.getX();
        double distanceToTop = -FUEL_RADIUS - robotWidth / 2 - relativePos.getY();
        double distanceToBottom = -FUEL_RADIUS - robotWidth / 2 + relativePos.getY();

        // not inside robot
        if (distanceToLeft > 0 || distanceToRight > 0 || distanceToTop > 0 || distanceToBottom > 0) return;

        Translation2d posOffset;
        // find minimum distance to side and send corresponding collision response
        if (relativePos.getX() <= robotLength / 2
                || (distanceToLeft >= distanceToRight
                        && distanceToLeft >= distanceToTop
                        && distanceToLeft >= distanceToBottom)) {
            posOffset = new Translation2d(distanceToLeft, 0);
        } else if (fuel.pos.getX() >= robotLength / 2
                || (distanceToRight >= distanceToLeft
                        && distanceToRight >= distanceToTop
                        && distanceToRight >= distanceToBottom)) {
            posOffset = new Translation2d(-distanceToRight, 0);
        } else if (fuel.pos.getY() >= robotWidth / 2
                || (distanceToTop >= distanceToLeft
                        && distanceToTop >= distanceToRight
                        && distanceToTop >= distanceToBottom)) {
            posOffset = new Translation2d(0, -distanceToTop);
        } else {
            posOffset = new Translation2d(0, distanceToBottom);
        }

        posOffset = posOffset.rotateBy(robot.getRotation());
        fuel.pos = fuel.pos.plus(new Translation3d(posOffset));
        Translation2d normal = posOffset.div(posOffset.getNorm());
        if (fuel.vel.toTranslation2d().dot(normal) < 0)
            fuel.addImpulse(
                    new Translation3d(normal.times(-fuel.vel.toTranslation2d().dot(normal) * (1 + ROBOT_COR))));
        if (robotVel.dot(normal) > 0) fuel.addImpulse(new Translation3d(normal.times(robotVel.dot(normal))));
    }

    private void handleRobotCollisions(ArrayList<Fuel> fuels) {
        Pose2d robot = robotSupplier.get();
        ChassisSpeeds speeds = robotSpeedsSupplier.get();
        Translation2d robotVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        Translation3d[][] bumperLines = getRobotBumperLines(robot);

        for (Fuel fuel : fuels) {
            handleRobotCollision(fuel, robot, robotVel, bumperLines);
        }
    }

    private void handleIntakes(ArrayList<Fuel> fuels) {
        Pose2d robot = robotSupplier.get();
        for (SimIntake intake : intakes) {
            for (int i = 0; i < fuels.size(); i++) {
                if (intake.shouldIntake(fuels.get(i), robot)) {
                    fuels.remove(i);
                    i--;
                }
            }
        }
    }

    public void registerIntake(
            double xMin, double xMax, double yMin, double yMax, BooleanSupplier ableToIntake, Runnable intakeCallback) {
        intakes.add(new SimIntake(xMin, xMax, yMin, yMax, ableToIntake, intakeCallback));
    }

    public void registerIntake(double xMin, double xMax, double yMin, double yMax, BooleanSupplier ableToIntake) {
        registerIntake(xMin, xMax, yMin, yMax, ableToIntake, () -> {});
    }

    public void registerIntake(double xMin, double xMax, double yMin, double yMax, Runnable intakeCallback) {
        registerIntake(xMin, xMax, yMin, yMax, () -> true, intakeCallback);
    }

    public void registerIntake(double xMin, double xMax, double yMin, double yMax) {
        registerIntake(xMin, xMax, yMin, yMax, () -> true, () -> {});
    }

    private static class Hub {
        public static final Hub BLUE_HUB =
                new Hub(new Translation2d(4.61, FIELD_WIDTH / 2), new Translation3d(5.3, FIELD_WIDTH / 2, 0.89), 1);
        public static final Hub RED_HUB = new Hub(
                new Translation2d(FIELD_LENGTH - 4.61, FIELD_WIDTH / 2),
                new Translation3d(FIELD_LENGTH - 5.3, FIELD_WIDTH / 2, 0.89),
                -1);

        private static final double ENTRY_HEIGHT = 1.83;
        private static final double ENTRY_RADIUS = 0.56;

        public static final double SIDE = 1.2;

        private final Translation2d center;
        private final Translation3d exit;
        private final int exitVelXMult;

        private int score = 0;

        private Hub(Translation2d center, Translation3d exit, int exitVelXMult) {
            this.center = center;
            this.exit = exit;
            this.exitVelXMult = exitVelXMult;
        }

        public void handleHubInteraction(Fuel fuel) {
            if (didFuelScore(fuel)) {
                fuel.pos = exit;
                fuel.vel = getDispersalVelocity();
                score++;
            }
        }

        private boolean didFuelScore(Fuel fuel) {
            return fuel.pos.toTranslation2d().getDistance(center) <= ENTRY_RADIUS
                    && fuel.pos.getZ() <= ENTRY_HEIGHT
                    && fuel.pos.minus(fuel.vel.times(PERIOD / subticks)).getZ() > ENTRY_HEIGHT;
        }

        private Translation3d getDispersalVelocity() {
            return new Translation3d(exitVelXMult * (Math.random() + 0.1) * 1.5, Math.random() * 2 - 1, 0);
        }

        public void resetScore() {
            score = 0;
        }

        public int getScore() {
            return score;
        }

        public Translation2d fuelCollideSide(Fuel fuel) {
            if (fuel.pos.getZ() > ENTRY_HEIGHT - 0.1) return new Translation2d(); // above hub
            double distanceToLeft = center.getX() - SIDE / 2 - FUEL_RADIUS - fuel.pos.getX();
            double distanceToRight = fuel.pos.getX() - center.getX() - SIDE / 2 - FUEL_RADIUS;
            double distanceToTop = center.getY() - SIDE / 2 - FUEL_RADIUS - fuel.pos.getY();
            double distanceToBottom = fuel.pos.getY() - center.getY() - SIDE / 2 - FUEL_RADIUS;

            // not inside hub
            if (distanceToLeft > 0 || distanceToRight > 0 || distanceToTop > 0 || distanceToBottom > 0)
                return new Translation2d();

            // find minimum distance to side and send corresponding collision response
            if (fuel.pos.getX() < center.getX() - SIDE / 2
                    || (distanceToLeft >= distanceToRight
                            && distanceToLeft >= distanceToTop
                            && distanceToLeft >= distanceToBottom)) {
                return new Translation2d(distanceToLeft, 0);
            } else if (fuel.pos.getX() >= center.getX() + SIDE / 2
                    || (distanceToRight >= distanceToLeft
                            && distanceToRight >= distanceToTop
                            && distanceToRight >= distanceToBottom)) {
                return new Translation2d(-distanceToRight, 0);
            } else if (fuel.pos.getY() > center.getY() + SIDE / 2
                    || (distanceToTop >= distanceToLeft
                            && distanceToTop >= distanceToRight
                            && distanceToTop >= distanceToBottom)) {
                return new Translation2d(0, -distanceToTop);
            } else {
                return new Translation2d(0, distanceToBottom);
            }
        }
    }

    private class SimIntake {
        double xMin, xMax, yMin, yMax;
        BooleanSupplier ableToIntake;
        Runnable callback;

        private SimIntake(
                double xMin,
                double xMax,
                double yMin,
                double yMax,
                BooleanSupplier ableToIntake,
                Runnable intakeCallback) {
            this.xMin = xMin;
            this.xMax = xMax;
            this.yMin = yMin;
            this.yMax = yMax;
            this.ableToIntake = ableToIntake;
            this.callback = intakeCallback;
        }

        private boolean shouldIntake(Fuel fuel, Pose2d robotPose) {
            if (!ableToIntake.getAsBoolean() || fuel.pos.getZ() > bumperHeight) return false;

            Translation2d fuelRelativePos = new Pose2d(fuel.pos.toTranslation2d(), Rotation2d.kZero)
                    .relativeTo(robotPose)
                    .getTranslation();

            boolean result = fuelRelativePos.getX() >= xMin
                    && fuelRelativePos.getX() <= xMax
                    && fuelRelativePos.getY() >= yMin
                    && fuelRelativePos.getY() <= yMax;
            if (result) {
                callback.run();
            }
            return result;
        }
    }
}
