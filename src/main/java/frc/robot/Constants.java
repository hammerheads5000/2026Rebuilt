// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayTopic;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.turret.TurretCalculator.ShotData;
import frc.robot.util.TunableControls.ControlConstants;
import frc.robot.util.TunableControls.TunableControlConstants;
import java.io.IOException;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always
 * "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics
 * sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode SIM_MODE = Mode.SIM;
    public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : SIM_MODE;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }

    public static final CANBus CAN_FD_BUS = new CANBus("Bobby");
    public static final CANBus CAN_RIO_BUS = new CANBus("rio");

    public static final Time SIM_LOOP_PERIOD = Milliseconds.of(20);

    public static final NetworkTableInstance INST = NetworkTableInstance.getDefault();

    public static final Voltage LOW_BATTERY_VOLTAGE = Volts.of(12.4);

    public static class Dimensions {
        public static final Distance BUMPER_THICKNESS = Inches.of(3); // frame to edge of bumper
        public static final Distance BUMPER_HEIGHT = Inches.of(7); // height from floor to top of bumper
        public static final Distance FRAME_SIZE_Y = Inches.of(26.25); // left to right (y-axis)
        public static final Distance FRAME_SIZE_X = Inches.of(28.75); // front to back (x-axis)

        public static final Distance FULL_WIDTH = FRAME_SIZE_Y.plus(BUMPER_THICKNESS.times(2));
        public static final Distance FULL_LENGTH = FRAME_SIZE_X.plus(BUMPER_THICKNESS.times(2));
    }

    public static class ControllerConstants {
        public static final double CONTROLLER_DEADBAND = 0.225;
        public static final double CONTROLLER_RUMBLE = 0.3;
    }

    public static class SwerveConstants {
        public static final LinearVelocity DEFAULT_DRIVE_SPEED = MetersPerSecond.of(3.2);
        public static final AngularVelocity DEFAULT_ROT_SPEED = RotationsPerSecond.of(0.75);

        public static final LinearVelocity FAST_DRIVE_SPEED = MetersPerSecond.of(4.5);
        public static final AngularVelocity FAST_ROT_SPEED = RotationsPerSecond.of(2);

        public static final LinearAcceleration MAX_TELEOP_ACCEL = MetersPerSecondPerSecond.of(25);

        public static final AngularVelocity MAX_MODULE_ROT_SPEED = RotationsPerSecond.of(5);

        private static final Distance MODULE_DISTANCE_Y = Inches.of(20.75); // left to right
        private static final Distance MODULE_DISTANCE_X = Inches.of(22.5); // front to back

        public static final Slot0Configs STEER_GAINS = new Slot0Configs()
                .withKP(1000)
                .withKI(0)
                .withKD(8)
                .withKS(6)
                .withKV(0)
                .withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

        public static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
                .withKP(2)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.237) // 0.11367, 0.1301, 0.15349, 0.16187 -> 0.140
                .withKV(0.733) // 0.13879, 0.13555, 0.13894, 0.13109 -> 0.136
                .withKA(0.0); // 0.016363, 0.016268, 0.0085342, 0.011084 -> 0.013

        private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.TorqueCurrentFOC;
        private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;

        private static final DriveMotorArrangement DRIVE_MOTOR_TYPE = DriveMotorArrangement.TalonFX_Integrated;
        private static final SteerMotorArrangement STEER_MOTOR_TYPE = SteerMotorArrangement.TalonFX_Integrated;

        private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

        private static final Current SLIP_CURRENT = Amps.of(95);

        private static final TalonFXConfiguration DRIVE_CONFIGS = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(80))
                        .withStatorCurrentLimitEnable(true))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

        private static final TalonFXConfiguration STEER_CONFIGS = new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(Amps.of(60))
                        .withStatorCurrentLimitEnable(true))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
                .withMotionMagic(
                        new MotionMagicConfigs().withMotionMagicExpo_kA(0.5).withMotionMagicExpo_kV(2.0));

        private static final CANcoderConfiguration ENCODER_CONFIGS = new CANcoderConfiguration();

        public static final Pigeon2Configuration PIGEON_CONFIGS =
                new Pigeon2Configuration().withMountPose(new MountPoseConfigs().withMountPoseYaw(Degrees.of(-90)));

        public static final LinearVelocity SPEED_AT_12V = MetersPerSecond.of(5.85); // theoretical free speed

        // Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
        private static final double COUPLE_RATIO = 3.375;

        private static final double DRIVE_GEAR_RATIO = 6.03; // R2 gear ratio
        private static final double STEER_GEAR_RATIO = 26.09090909090909;
        private static final Distance WHEEL_RADIUS = Inches.of(1.985);

        public static final int PIGEON_ID = 6;

        // SIMULATION inertia
        private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
        private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.01);
        // SIMULATION voltage necessary to overcome friction
        private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
        private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

        private static final double WHEEL_COF = 2.255;

        public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
                .withCANBusName(CAN_FD_BUS.getName())
                .withPigeon2Id(PIGEON_ID)
                .withPigeon2Configs(PIGEON_CONFIGS);

        private static final SwerveModuleConstantsFactory<
                        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                CONSTANT_CREATOR = new SwerveModuleConstantsFactory<
                                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                        .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                        .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                        .withCouplingGearRatio(COUPLE_RATIO)
                        .withWheelRadius(WHEEL_RADIUS)
                        .withSteerMotorGains(STEER_GAINS)
                        .withDriveMotorGains(DRIVE_GAINS)
                        .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
                        .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
                        .withSlipCurrent(SLIP_CURRENT)
                        .withSpeedAt12Volts(SPEED_AT_12V)
                        .withDriveMotorType(DRIVE_MOTOR_TYPE)
                        .withSteerMotorType(STEER_MOTOR_TYPE)
                        .withFeedbackSource(STEER_FEEDBACK_TYPE)
                        .withDriveMotorInitialConfigs(DRIVE_CONFIGS)
                        .withSteerMotorInitialConfigs(STEER_CONFIGS)
                        .withEncoderInitialConfigs(ENCODER_CONFIGS)
                        .withSteerInertia(STEER_INERTIA)
                        .withDriveInertia(DRIVE_INERTIA)
                        .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
                        .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

        public static class FrontLeft {
            private static final int DRIVE_ID = 16;
            private static final int STEER_ID = 15;
            private static final int ENCODER_ID = 14;
            private static final Angle ENCODER_OFFSET = Rotations.of(0.28564453125);
            private static final boolean STEER_INVERTED = false;
            private static final boolean ENCODER_INVERTED = false;
            private static final boolean DRIVE_INVERTED = false;

            public static final Distance X_POS = MODULE_DISTANCE_X.div(2);
            public static final Distance Y_POS = MODULE_DISTANCE_Y.div(2);

            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                    MODULE_CONSTANTS = CONSTANT_CREATOR.createModuleConstants(
                            STEER_ID,
                            DRIVE_ID,
                            ENCODER_ID,
                            ENCODER_OFFSET,
                            X_POS,
                            Y_POS,
                            DRIVE_INVERTED,
                            STEER_INVERTED,
                            ENCODER_INVERTED);
        }

        public static class FrontRight {
            private static final int DRIVE_ID = 9;
            private static final int STEER_ID = 10;
            private static final int ENCODER_ID = 11;
            private static final Angle ENCODER_OFFSET = Rotations.of(-0.109375);
            private static final boolean STEER_INVERTED = false;
            private static final boolean ENCODER_INVERTED = false;
            private static final boolean DRIVE_INVERTED = true;

            public static final Distance X_POS = MODULE_DISTANCE_X.div(2);
            public static final Distance Y_POS = MODULE_DISTANCE_Y.div(-2);

            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                    MODULE_CONSTANTS = CONSTANT_CREATOR.createModuleConstants(
                            STEER_ID,
                            DRIVE_ID,
                            ENCODER_ID,
                            ENCODER_OFFSET,
                            X_POS,
                            Y_POS,
                            DRIVE_INVERTED,
                            STEER_INVERTED,
                            ENCODER_INVERTED);
        }

        public static class BackLeft {
            private static final int DRIVE_ID = 18;
            private static final int STEER_ID = 19;
            private static final int ENCODER_ID = 20;
            private static final Angle ENCODER_OFFSET = Rotations.of(-0.188232421875);
            private static final boolean STEER_INVERTED = false;
            private static final boolean ENCODER_INVERTED = false;
            private static final boolean DRIVE_INVERTED = false;

            public static final Distance X_POS = MODULE_DISTANCE_X.div(-2);
            public static final Distance Y_POS = MODULE_DISTANCE_Y.div(2);

            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                    MODULE_CONSTANTS = CONSTANT_CREATOR.createModuleConstants(
                            STEER_ID,
                            DRIVE_ID,
                            ENCODER_ID,
                            ENCODER_OFFSET,
                            X_POS,
                            Y_POS,
                            DRIVE_INVERTED,
                            STEER_INVERTED,
                            ENCODER_INVERTED);
        }

        public static class BackRight {
            private static final int DRIVE_ID = 5;
            private static final int STEER_ID = 4;
            private static final int ENCODER_ID = 3;
            private static final Angle ENCODER_OFFSET = Rotations.of(-0.134033203125);
            private static final boolean STEER_INVERTED = false;
            private static final boolean ENCODER_INVERTED = false;
            private static final boolean DRIVE_INVERTED = true;

            public static final Distance X_POS = MODULE_DISTANCE_X.div(-2);
            public static final Distance Y_POS = MODULE_DISTANCE_Y.div(-2);

            public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
                    MODULE_CONSTANTS = CONSTANT_CREATOR.createModuleConstants(
                            STEER_ID,
                            DRIVE_ID,
                            ENCODER_ID,
                            ENCODER_OFFSET,
                            X_POS,
                            Y_POS,
                            DRIVE_INVERTED,
                            STEER_INVERTED,
                            ENCODER_INVERTED);
        }

        public static final Supplier<Translation2d[]> GET_MODULE_POSITIONS = () -> new Translation2d[] {
            new Translation2d(SwerveConstants.FrontLeft.X_POS, SwerveConstants.FrontLeft.Y_POS),
            new Translation2d(SwerveConstants.FrontRight.X_POS, SwerveConstants.FrontRight.Y_POS),
            new Translation2d(SwerveConstants.BackLeft.X_POS, SwerveConstants.BackLeft.Y_POS),
            new Translation2d(SwerveConstants.BackRight.X_POS, SwerveConstants.BackRight.Y_POS),
        };

        public static final Frequency ODOMETRY_UPDATE_FREQ = Hertz.of(250); // 0 Hz = default 250 Hz for CAN FD
        public static final Matrix<N3, N1> ODOMETRY_STD_DEV = VecBuilder.fill(0.02, 0.02, 0.01);

        public static final DriveRequestType DRIVE_REQUEST_TYPE = DriveRequestType.Velocity;
        public static final SteerRequestType STEER_REQUEST_TYPE = SteerRequestType.MotionMagicExpo;

        public static final LinearVelocity LINEAR_VEL_DEADBAND = MetersPerSecond.of(0.02);
        public static final AngularVelocity ANGLULAR_VEL_DEADBAND = DegreesPerSecond.of(1);

        // Characterization
        public static final Time FF_START_DELAY = Seconds.of(2.0);
        public static final Velocity<VoltageUnit> FF_RAMP_RATE = Volts.of(0.1).per(Second);
        public static final AngularVelocity FF_WHEEL_RADIUS_MAX_VELOCITY = RadiansPerSecond.of(0.25);
        public static final AngularAcceleration FF_WHEEL_RADIUS_RAMP_RATE = RadiansPerSecondPerSecond.of(0.05);

        // Alignment
        private static final ControlConstants TRENCH_TRANSLATION_BASE_CONSTANTS =
                new ControlConstants().withPID(8, 0, 0.05).withTolerance(0.05);
        private static final ControlConstants ROTATION_BASE_CONSTANTS =
                new ControlConstants().withPID(5, 0, 0).withTolerance(0.08).withContinuous(-Math.PI, Math.PI);

        public static final TunableControlConstants TRENCH_TRANSLATION_CONSTANTS =
                new TunableControlConstants("Swerve/Trench Translation", TRENCH_TRANSLATION_BASE_CONSTANTS);
        public static final TunableControlConstants ROTATION_CONSTANTS =
                new TunableControlConstants("Swerve/Rotation", ROTATION_BASE_CONSTANTS);
        public static final Time TRENCH_ALIGN_TIME = Seconds.of(0.5);
        public static final Time BUMP_ALIGN_TIME = Seconds.of(0.3);
    }

    public static class TurretConstants {
        public static final int TURN_ID = 22;
        public static final int HOOD_ID = 26;
        public static final int FLYWHEEL_ID = 24;
        public static final int FLYWHEEL_FOLLOWER_ID = 25;
        public static final int ENCODER_ID = 23;

        public static final double ENCODER_TO_TURRET_RATIO = 180.0 / 27 * 1.0 / 23; // 180:27 big gear, 1:23 cycloidal
        public static final double TURN_TO_TURRET_RATIO = 28.0 / 12 * 180.0 / 10; // 28:14 belt, 180:10 big gear
        public static final double HOOD_MOTOR_RATIO =
                50.0 / 14 * 22.0 / 20 * 156.0 / 10; // 50:14 gear, 22:20 belt, 156:10 rack

        public static final Slot0Configs TURN_GAINS =
                new Slot0Configs().withKP(300).withKD(0.0).withKS(2).withKV(0.0);

        public static final Slot0Configs HOOD_GAINS =
                new Slot0Configs().withKP(800).withKD(5).withKS(0.28);

        public static final Slot0Configs FLYWHEEL_GAINS =
                new Slot0Configs().withKP(12).withKD(0.0).withKS(6).withKV(0.04);

        public static final CurrentLimitsConfigs TURN_CURRENT_LIMITS =
                new CurrentLimitsConfigs().withSupplyCurrentLowerLimit(30);

        public static final CurrentLimitsConfigs HOOD_CURRENT_LIMITS =
                new CurrentLimitsConfigs().withSupplyCurrentLowerLimit(30);

        public static final CurrentLimitsConfigs FLYWHEEL_CURRENT_LIMITS =
                new CurrentLimitsConfigs().withStatorCurrentLimit(100);

        public static final MotorOutputConfigs TURN_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        public static final FeedbackConfigs TURN_FEEDBACK_CONFIGS = new FeedbackConfigs()
                .withFeedbackRemoteSensorID(ENCODER_ID)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withSensorToMechanismRatio(ENCODER_TO_TURRET_RATIO)
                .withRotorToSensorRatio(TURN_TO_TURRET_RATIO / ENCODER_TO_TURRET_RATIO)
                .withFeedbackRotorOffset(0)
                .withVelocityFilterTimeConstant(0.01);

        public static final MotorOutputConfigs HOOD_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        public static final MotorOutputConfigs FLYWHEEL_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);

        public static final MotorOutputConfigs FLYWHEEL_FOLLOWER_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);

        public static final MagnetSensorConfigs ENCODER_CONFIGS = new MagnetSensorConfigs()
                .withMagnetOffset(-0.2724609375)
                .withAbsoluteSensorDiscontinuityPoint(0.5)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive);

        public static final FeedbackConfigs FLYWHEEL_FEEDBACK_CONFIGS =
                new FeedbackConfigs().withVelocityFilterTimeConstant(Seconds.of(0.01));

        public static final Current BANG_BANG_AMPS = Amps.of(100);

        public static final Distance DISTANCE_ABOVE_FUNNEL = Inches.of(20); // how high to clear the funnel
        public static final Distance APEX = Inches.of(130);
        public static final Transform3d ROBOT_TO_TURRET_TRANSFORM =
                new Transform3d(new Translation3d(Inches.zero(), Inches.zero(), Inches.of(18)), Rotation3d.kZero);
        public static final Distance FLYWHEEL_RADIUS = Inches.of(2);
        public static final Distance SHOOT_RADIUS = Inches.of(1);
        public static final int LOOKAHEAD_ITERATIONS = 3;

        public static final Angle MIN_TURN_ANGLE = Degrees.of(-230);
        public static final Angle MAX_TURN_ANGLE = Degrees.of(210);
        public static final Angle TURNAROUND_ZONE = Degrees.of(30);

        public static final Distance EXTRA_DUCK_DISTANCE = Meters.of(0.3);
        public static final Time DUCK_TIME = Seconds.of(0.2);

        public static final Angle MIN_HOOD_ANGLE = Degrees.of(6.24);
        public static final Angle MAX_HOOD_ANGLE = Degrees.of(47);

        public static final Current HOOD_STALL_CURRENT = Amps.of(10);
        public static final AngularVelocity HOOD_STALL_ANGULAR_VELOCITY = RadiansPerSecond.of(0.3);
        public static final Voltage HOOD_ZEROING_VOLTAGE = Volts.of(-1);

        public static final AngularVelocity FLYWHEEL_FUDGE_AMOUNT = RPM.of(10);

        public static final AngularVelocity FLYWHEEL_SCORING_OVERRIDE = RPM.of(2700);
        public static final Angle HOOD_SCORING_OVERRIDE = Degrees.of(25);

        public static final AngularVelocity FLYWHEEL_PASSING_OVERRIDE = RPM.of(2800);
        public static final Angle HOOD_PASSING_OVERRIDE = Degrees.of(27);

        public static final Translation3d PASSING_SPOT_LEFT = new Translation3d(
                Inches.of(90), FieldConstants.FIELD_WIDTH.div(2).plus(Inches.of(85)), Inches.zero());
        public static final Translation3d PASSING_SPOT_CENTER =
                new Translation3d(Inches.of(90), FieldConstants.FIELD_WIDTH.div(2), Inches.zero());
        public static final Translation3d PASSING_SPOT_RIGHT = new Translation3d(
                Inches.of(90), FieldConstants.FIELD_WIDTH.div(2).minus(Inches.of(85)), Inches.zero());

        public static final InterpolatingTreeMap<Double, ShotData> SHOT_MAP =
                new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotData::interpolate);

        public static final InterpolatingTreeMap<Double, ShotData> PASSING_MAP =
                new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotData::interpolate);

        public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();

        public static final InterpolatingDoubleTreeMap PASS_TOF_MAP = new InterpolatingDoubleTreeMap();

        static {
            SHOT_MAP.put(6.3, new ShotData(RPM.of(3720), Degrees.of(42)));
            TOF_MAP.put(6.3, 1.5);

            SHOT_MAP.put(5.5, new ShotData(RPM.of(3720), Degrees.of(39)));
            TOF_MAP.put(5.5, 1.46);

            SHOT_MAP.put(5.18, new ShotData(RPM.of(3700), Degrees.of(35)));
            TOF_MAP.put(5.18, 1.42);

            SHOT_MAP.put(4.72, new ShotData(RPM.of(3600), Degrees.of(33)));
            TOF_MAP.put(4.72, 1.42);

            SHOT_MAP.put(4.24, new ShotData(RPM.of(3500), Degrees.of(32)));
            TOF_MAP.put(4.24, 1.41);

            SHOT_MAP.put(3.87, new ShotData(RPM.of(3450), Degrees.of(31)));
            TOF_MAP.put(3.87, 1.41);

            SHOT_MAP.put(3.29, new ShotData(RPM.of(3300), Degrees.of(29)));
            TOF_MAP.put(3.29, 1.30);

            SHOT_MAP.put(2.84, new ShotData(RPM.of(3260), Degrees.of(26)));
            TOF_MAP.put(2.84, 1.30);

            SHOT_MAP.put(2.45, new ShotData(RPM.of(3100), Degrees.of(25)));
            TOF_MAP.put(2.45, 1.25);

            SHOT_MAP.put(2.09, new ShotData(RPM.of(3000), Degrees.of(22.5)));
            TOF_MAP.put(2.09, 1.17);

            SHOT_MAP.put(1.73, new ShotData(RPM.of(2900), Degrees.of(20)));
            TOF_MAP.put(1.73, 1.17);

            SHOT_MAP.put(1.37, new ShotData(RPM.of(2800), Degrees.of(17)));
            TOF_MAP.put(1.37, 1.09);

            PASSING_MAP.put(12.6, new ShotData(RPM.of(5000), Degrees.of(47)));
            PASS_TOF_MAP.put(12.6, 1.5);

            PASSING_MAP.put(7.7, new ShotData(RPM.of(3600), Degrees.of(47)));
            PASS_TOF_MAP.put(7.7, 1.4);

            PASSING_MAP.put(5.5, new ShotData(RPM.of(3400), Degrees.of(47)));
            PASS_TOF_MAP.put(5.5, 1.3);

            PASSING_MAP.put(1.0, new ShotData(RPM.of(700), Degrees.of(47)));
            PASS_TOF_MAP.put(1.0, 1.);
        }

        public static final Time ACTIVE_PRESHOOT_TIME = Seconds.of(2);
    }

    public static class IntakeConstants {
        public static final int LEFT_RACK_ID = 21;
        public static final int RIGHT_RACK_ID = 2;
        public static final int LEFT_SPIN_ID = 28;
        public static final int RIGHT_SPIN_ID = 27;

        public static final double LEFT_ROTOR_TO_PINION_RATIO = 34.0 / 12; // 34:12 gear
        public static final double RIGHT_ROTOR_TO_PINION_RATIO = 50.0 / 17; // 50:17 belt
        public static final Distance PINION_PITCH_RADIUS = Inches.of(0.5);

        // public static final Slot0Configs RIGHT_RACK_GAINS = new Slot0Configs()
        //         .withKP(3.0)
        //         .withKD(0.1)
        //         .withKA(0.0)
        //         .withKV(0.23)
        //         .withKS(0.4);

        public static final Slot0Configs LEFT_RACK_GAINS = new Slot0Configs()
                .withKP(21)
                .withKD(0.15)
                .withKA(0.0)
                .withKV(3.3)
                .withKS(1.7);

        // public static final Slot1Configs RACK_DIFF_GAINS = new Slot1Configs().withKP(0.5);

        public static final MotorOutputConfigs RIGHT_RACK_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive);

        public static final MotorOutputConfigs LEFT_RACK_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive);

        public static final MotorOutputConfigs SPIN_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.CounterClockwise_Positive);

        public static final CurrentLimitsConfigs RACK_CURRENT_LIMITS =
                new CurrentLimitsConfigs().withStatorCurrentLimit(100).withSupplyCurrentLowerLimit(30);

        public static final CurrentLimitsConfigs SPIN_CURRENT_LIMITS =
                new CurrentLimitsConfigs().withStatorCurrentLimit(65).withSupplyCurrentLowerLimit(30);

        public static final MotionMagicConfigs RACK_MOTION_MAGIC = new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(RotationsPerSecond.of(240))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(400));

        public static final Distance STOW_POS = Inches.of(0);
        public static final Distance DEPLOY_POS = Inches.of(11);
        public static final Voltage SPIN_VOLTAGE = Volts.of(12);
        public static final Voltage REVERSE_SPIN_VOLTAGE = Volts.of(-2);
        public static final Voltage UNJAM_SPIN_VOLTAGE = Volts.of(10);
        public static final Distance STOW_TOLERANCE = Inches.of(0.5);
        public static final Distance DEPLOY_TOLERANCE = Inches.of(1);
        public static final Distance DECOUPLE_DISTANCE = Inches.of(3);

        public static final LinearVelocity MIN_SWITCH_ROBOT_VELOCITY = MetersPerSecond.of(0.2);

        public static final Current RACK_STALL_CURRENT = Amps.of(40);
        public static final LinearVelocity RACK_STALL_VEL = InchesPerSecond.of(1);
        public static final Voltage ZEROING_VOLTAGE = Volts.of(-5);

        public static final Current SPIN_STALL_CURRENT = Amps.of(20);
        public static final AngularVelocity SPIN_STALL_ANGULAR_VELOCITY = RadiansPerSecond.of(0.5);
    }

    public static class IndexerConstants {
        public static final int SPIN_ID = 17;
        public static final int FEED_ID = 8;

        public static final MotorOutputConfigs SPIN_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive);

        public static final OpenLoopRampsConfigs SPIN_RAMPS =
                new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(Seconds.of(1));

        public static final MotorOutputConfigs FEED_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive);

        public static final CurrentLimitsConfigs SPIN_CURRENT_LIMITS =
                new CurrentLimitsConfigs().withSupplyCurrentLowerLimit(30).withStatorCurrentLimit(50);

        public static final CurrentLimitsConfigs FEED_CURRENT_LIMITS =
                new CurrentLimitsConfigs().withSupplyCurrentLowerLimit(30).withStatorCurrentLimit(100);

        public static final Voltage SPIN_VOLTAGE = Volts.of(4);
        public static final Voltage FEED_VOLTAGE = Volts.of(11);
        public static final Voltage UNJAM_SPIN_VOLTAGE = Volts.of(-2);
        public static final Voltage UNJAM_FEED_VOLTAGE = Volts.of(-5);

        public static final AngularVelocity SPIN_STALL_ANGULAR_VELOCITY = RPM.of(1000);
        public static final Current SPIN_STALL_CURRENT = Amps.of(30);
    }

    public static class ClimberConstants {
        public static final int FRONT_ID = 13;
        public static final int BACK_ID = 1;

        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIGS =
                new CurrentLimitsConfigs().withStatorCurrentLimit(80);

        public static final MotorOutputConfigs FRONT_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        public static final MotorOutputConfigs BACK_OUTPUT_CONFIGS = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        public static final Voltage CLIMB_VOLTAGE = Volts.of(-12);
        public static final Voltage STOW_VOLTAGE = Volts.of(-10);
        public static final Voltage STOW_SLOW_VOLTAGE = Volts.of(-1);
        public static final Voltage EXTEND_VOLTAGE = Volts.of(3);
        public static final Voltage ZERO_VOLTAGE = Volts.of(-1);

        public static final Current STALL_CURRENT = Amps.of(20);
        public static final AngularVelocity STALL_ANGULAR_VELOCITY = RadiansPerSecond.of(6);

        public static final Angle CLIMB_POSITION = Rotations.of(25);
        public static final Angle AUTO_CLIMB_POSITION = Rotations.of(30);
        public static final Angle STOW_POSITION = Rotations.of(0.2);
        public static final Angle STOW_SLOW_POSITION = Rotations.of(15);
        public static final Angle EXTEND_POSITION_FRONT = Rotations.of(40);
        public static final Angle EXTEND_POSITION_BACK = Rotations.of(43);

        // volts / rotation diff
        public static final double DIFF_KP = 0.0;

        private static final ControlConstants CLIMB_ALIGN_BASE_CONSTANTS_TRANSLATION =
                new ControlConstants().withPID(5, 0, 0).withTolerance(0.02);

        public static final TunableControlConstants CLIMB_ALIGN_CONSTANTS_TRANSLATION =
                new TunableControlConstants("Climber/AlignTranslation", CLIMB_ALIGN_BASE_CONSTANTS_TRANSLATION);

        private static final ControlConstants CLIMB_ALIGN_BASE_CONSTANTS_ROTATION = new ControlConstants()
                .withPID(3, 0, 0)
                .withTolerance(Degrees.of(3).in(Radians))
                .withContinuous(-Math.PI, Math.PI);

        public static final TunableControlConstants CLIMB_ALIGN_CONSTANTS_ROTATION =
                new TunableControlConstants("Climber/AlignRotation", CLIMB_ALIGN_BASE_CONSTANTS_ROTATION);

        public static enum ClimbPosition {
            FRONT_LEFT(new Pose2d(1.54, 3.91, Rotation2d.kCW_90deg)),
            FRONT_RIGHT(new Pose2d(1.54, 3.50, Rotation2d.kCW_90deg)),
            BACK_LEFT(new Pose2d(0.67, 3.91, Rotation2d.kCCW_90deg)),
            BACK_RIGHT(new Pose2d(0.67, 3.50, Rotation2d.kCCW_90deg));

            private Pose2d pose;

            private ClimbPosition(Pose2d pose) {
                this.pose = pose;
            }

            public Pose2d getPose() {
                return pose;
            }
        }
    }

    public static class VisionConstants {
        // Standard deviation baselines for 1 meter distance to single tag
        public static final double[] LINEAR_STD_DEV_BASELINES = {0.2, 0.2, 0.07, 0.3, 0.3, 0.3}; // Meters
        public static final double ANGULAR_STD_DEV_BASELINE = 1.0; // Radians

        public static final String[] CAMERA_NAMES = {
            "Arducam_OV9281_Front_Left", "Arducam_OV9281_Front_Right",
            "Arducam_OV9281_Front_Center", "Arducam_OV9281_Back_Left",
            "Arducam_OV9281_Back_Right", "Arducam_OV9281_Back_Center"
        };

        public static final double MAX_AMBIGUITY = 0.3;
        public static final Distance MAX_Z_HEIGHT = Meters.of(0.2);

        public static final AprilTagFieldLayout APRIL_TAGS;

        static {
            AprilTagFieldLayout tryAprilTags;
            try {
                tryAprilTags =
                        new AprilTagFieldLayout(Filesystem.getDeployDirectory().toPath() + "/apriltags.json");
                Logger.recordOutput("AprilTagLayoutType", "Custom");
            } catch (IOException e) {
                tryAprilTags = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
                Logger.recordOutput("AprilTagLayoutType", "Andymark");
            }
            APRIL_TAGS = tryAprilTags;
        }

        public static final Set<Integer> BLUE_HUB_TAG_IDS = Set.of(18, 19, 20, 21, 24, 25, 26, 27);
        public static final Set<Integer> RED_HUB_TAG_IDS = Set.of(2, 3, 4, 5, 8, 9, 10, 1);
        public static final Set<Integer> BLUE_CLIMB_TAG_IDS = Set.of(31, 32);
        public static final Set<Integer> RED_CLIMB_TAG_IDS = Set.of(15, 16);

        public static final double HUB_TAG_STD_DEV_BIAS = 0.08; // added to non-hub tags in hub mode
        public static final double CLIMB_TAG_STD_DEV_BIAS = 0.08; // added to non-climb tags in climb mode

        // Transforms from robot to cameras, (x forward, y left, z up), (roll, pitch,
        // yaw)
        public static final Transform3d[] CAMERA_TRANSFORMS = {
            new Transform3d(
                    new Translation3d(0.309, 0.317, 0.529),
                    new Rotation3d(Degrees.zero(), Degrees.of(-21), Degrees.of(65))),
            new Transform3d(
                    new Translation3d(0.309, -0.317, 0.529),
                    new Rotation3d(Degrees.zero(), Degrees.of(-21), Degrees.of(-65))),
            new Transform3d(
                    new Translation3d(0.352, 0, 0.529), new Rotation3d(Degrees.zero(), Degrees.of(-21), Degrees.of(0))),
            new Transform3d(
                    new Translation3d(-0.302, 0.313, 0.529),
                    new Rotation3d(Degrees.zero(), Degrees.of(-21), Degrees.of(90))),
            new Transform3d(
                    new Translation3d(-0.302, -0.313, 0.529),
                    new Rotation3d(Degrees.zero(), Degrees.of(-21), Degrees.of(-90))),
            new Transform3d(
                    new Translation3d(-0.352, 0, 0.529),
                    new Rotation3d(Degrees.zero(), Degrees.of(-21), Degrees.of(180)))
        };
    }

    public static class FieldConstants {
        public static final Distance FIELD_LENGTH = Inches.of(650.12);
        public static final Distance FIELD_WIDTH = Inches.of(316.64);

        public static final Distance ALLIANCE_ZONE = Inches.of(156.06);

        public static final Translation3d HUB_BLUE =
                new Translation3d(Inches.of(181.56), FIELD_WIDTH.div(2), Inches.of(56.4));
        public static final Translation3d HUB_RED =
                new Translation3d(FIELD_LENGTH.minus(Inches.of(181.56)), FIELD_WIDTH.div(2), Inches.of(56.4));
        public static final Distance FUNNEL_RADIUS = Inches.of(24);
        public static final Distance FUNNEL_HEIGHT = Inches.of(72 - 56.4);

        public static final Distance TRENCH_BUMP_X =
                Inches.of(181.56); // x position of the center of the trench and bump
        public static final Distance TRENCH_WIDTH = Inches.of(49.86); // y width of the trench
        public static final Distance TRENCH_BUMP_LENGTH = Inches.of(47); // x length of the trench and bump
        public static final Distance TRENCH_BAR_WIDTH = Inches.of(4); // x width of the trench bar
        public static final Distance TRENCH_BLOCK_WIDTH = Inches.of(12); // y width of block separating bump and trench
        public static final Distance BUMP_WIDTH = Inches.of(73); // y width of bump

        public static final Distance TRENCH_CENTER = TRENCH_WIDTH.div(2);
    }

    public static class AutoConstants {
        public static final LoggedNetworkString AUTO_SELECTION = new LoggedNetworkString("Autos/Selection");
        public static final StringArrayTopic AUTO_OPTIONS = INST.getStringArrayTopic("Autos/Auto Options");
        public static final DoubleArrayTopic AUTO_OPTION_TIMES = INST.getDoubleArrayTopic("Autos/Auto Option Times");
        public static final StringArrayTopic START_OPTIONS = INST.getStringArrayTopic("Autos/Start Options");
        public static final StructArrayTopic<Translation2d> TRAJECTORY =
                INST.getStructArrayTopic("Autos/Trajectory", Translation2d.struct);
        public static final DoubleArrayTopic TRAJECTORY_TIMESTAMPS =
                INST.getDoubleArrayTopic("Autos/Trajectory Timestamps");
        public static final DoubleTopic TIMESTAMP = INST.getDoubleTopic("Autos/timestamp");
        public static final LoggedNetworkBoolean DUMP_AT_START = new LoggedNetworkBoolean("Autos/Dump At Start", true);

        public static final RobotConfig PP_CONFIG = new RobotConfig(
                Pounds.of(130),
                KilogramSquareMeters.of(6.3),
                new ModuleConfig(
                        SwerveConstants.WHEEL_RADIUS,
                        SwerveConstants.SPEED_AT_12V,
                        SwerveConstants.WHEEL_COF,
                        DCMotor.getKrakenX60Foc(1),
                        SwerveConstants.DRIVE_GEAR_RATIO,
                        Amps.of(SwerveConstants.DRIVE_CONFIGS.CurrentLimits.StatorCurrentLimit),
                        1),
                SwerveConstants.GET_MODULE_POSITIONS.get());

        public static final PIDConstants PP_TRANSLATION_CONSTANTS = new PIDConstants(3, 0.05);
        public static final PIDConstants PP_ROTATION_CONSTANTS = new PIDConstants(2, 0.05);

        public static final PathConstraints CONSTRAINTS = new PathConstraints(
                MetersPerSecond.of(3),
                MetersPerSecondPerSecond.of(3.5),
                DegreesPerSecond.of(360),
                DegreesPerSecondPerSecond.of(540),
                Volts.of(12),
                false);

        public static final PathConstraints SCORING_CONSTRAINTS = new PathConstraints(
                MetersPerSecond.of(2.5),
                MetersPerSecondPerSecond.of(2.5),
                DegreesPerSecond.of(360),
                DegreesPerSecondPerSecond.of(360),
                Volts.of(12),
                false);

        public static final PathConstraints COLLECT_CONSTRAINTS = new PathConstraints(
                MetersPerSecond.of(2),
                MetersPerSecondPerSecond.of(3),
                DegreesPerSecond.of(360),
                DegreesPerSecondPerSecond.of(540),
                Volts.of(12),
                false);
        public static final LinearVelocity HANDOFF_VELOCITY = MetersPerSecond.of(2.5);
        public static final Time START_DUMP_TIME = Seconds.of(1.5);
        public static final Time START_SPIN_UP_TIME = Seconds.of(0.5);
        public static final Time CLIMB_TIME_REMAINING = Seconds.of(3);

        public static final Map<String, String> PREBUILT_AUTOS = Map.of(
                "Stay in Place", "",
                "Left Single Sweep",
                        "Trench Mid Start Left;Trench Mid Start Left to Trench Left;Collect Sweep and Return 1 Left!;Trench Left to Dump Left=4.0",
                "Right Single Sweep",
                        "Trench Mid Start Right;Trench Mid Start Right to Trench Right;Collect Sweep and Return 1 Right!;Trench Right to Dump Right=4.0",
                "Left Double Sweep",
                        "Trench Mid Start Left;Trench Mid Start Left to Trench Left;Collect Sideways 2 Left!;Trench Left to Dump Left=3.0;Dump Left to Trench Left;Collect Sweep 1 Left!;Trench Left to Dump Left=3.0",
                "Right Double Sweep",
                        "Trench Mid Start Right;Trench Mid Start Right to Trench Right;Collect Sideways 2 Right!;Trench Right to Dump Right=3.0;Dump Right to Trench Right;Collect Sweep 1 Right!;Trench Right to Dump Right=3.0",
                "Left Single Sweep + Climb",
                        "Trench Mid Start Left;Trench Mid Start Left to Trench Left;Collect Sweep and Return 1 Left!;Trench Left to Dump Left=3.0;Dump Left to Climb Left",
                "Right Single Sweep + Climb",
                        "Trench Mid Start Right;Trench Mid Start Right to Trench Right;Collect Sweep and Return 1 Right!;Trench Right to Dump Right=3.0;Dump Right to Climb Right",
                "Left Double Sweep + Climb",
                        "Trench Mid Start Left;Trench Mid Start Left to Trench Left;Collect Sideways 2 Left!;Trench Left to Dump Left=2.0;Dump Left to Trench Left;Collect Sweep 1 Left!;Trench Left to Dump Left=1.0;Dump Left to Climb Left",
                "Right Double Sweep + Climb",
                        "Trench Mid Start Right;Trench Mid Start Right to Trench Right;Collect Sideways 2 Right!;Trench Right to Dump Right=2.0;Dump Right to Trench Right;Collect Sweep 1 Right!;Trench Right to Dump Right=1.0;Dump Right to Climb Right");
    }

    private Constants() {}
}
