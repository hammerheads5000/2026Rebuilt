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
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.function.Supplier;

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

        public static final LinearAcceleration MAX_TELEOP_ACCEL = MetersPerSecondPerSecond.of(15);

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
                .withKP(0)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.208) // 0.11367, 0.1301, 0.15349, 0.16187 -> 0.140
                .withKV(0.726) // 0.13879, 0.13555, 0.13894, 0.13109 -> 0.136
                .withKA(0.0); // 0.016363, 0.016268, 0.0085342, 0.011084 -> 0.013

        private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.TorqueCurrentFOC;
        private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;

        private static final DriveMotorArrangement DRIVE_MOTOR_TYPE = DriveMotorArrangement.TalonFX_Integrated;
        private static final SteerMotorArrangement STEER_MOTOR_TYPE = SteerMotorArrangement.TalonFX_Integrated;

        private static final SteerFeedbackType STEER_FEEDBACK_TYPE = SteerFeedbackType.FusedCANcoder;

        private static final Current SLIP_CURRENT = Amps.of(80.0); // NEEDS TUNING

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

        private static final double DRIVE_GEAR_RATIO = 5.2734375;
        private static final double STEER_GEAR_RATIO = 26.09090909090909;
        private static final Distance WHEEL_RADIUS = Inches.of(1.985);

        public static final int PIGEON_ID = 6;

        // SIMULATION inertia
        private static final MomentOfInertia STEER_INERTIA = KilogramSquareMeters.of(0.01);
        private static final MomentOfInertia DRIVE_INERTIA = KilogramSquareMeters.of(0.01);
        // SIMULATION voltage necessary to overcome friction
        private static final Voltage STEER_FRICTION_VOLTAGE = Volts.of(0.2);
        private static final Voltage DRIVE_FRICTION_VOLTAGE = Volts.of(0.2);

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
            private static final int DRIVE_ID = 19;
            private static final int STEER_ID = 20;
            private static final int ENCODER_ID = 21;
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
    }

    public static class TurretConstants {
        public static final int TURN_ID = 0;
        public static final int HOOD_ID = 0;
        public static final int FLYWHEEL_ID = 0;
        public static final int SHOOT_ID = 0;
        public static final int ENCODER1_ID = 0;
        public static final int ENCODER2_ID = 0;

        public static final Slot0Configs TURN_GAINS =
                new Slot0Configs().withKP(0.0).withKD(0.0).withKS(0.0);

        public static final Slot0Configs HOOD_GAINS =
                new Slot0Configs().withKP(0.0).withKD(0.0).withKS(0.0);

        public static final Slot0Configs FLYWHEEL_GAINS =
                new Slot0Configs().withKP(0.3).withKD(0).withKS(0);

        public static final CurrentLimitsConfigs TURN_CURRENT_LIMITS = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(30)
                .withSupplyCurrentLowerTime(1)
                .withSupplyCurrentLowerLimit(40);

        public static final Distance DISTANCE_ABOVE_FUNNEL = Inches.of(20); // how high to clear the funnel
        public static final Distance APEX = Inches.of(130);
        public static final Transform3d ROBOT_TO_TURRET_TRANSFORM =
                new Transform3d(new Translation3d(Inches.zero(), Inches.zero(), Inches.of(18)), Rotation3d.kZero);
        public static final Distance FLYWHEEL_RADIUS = Inches.of(2);
        public static final Distance SHOOT_RADIUS = Inches.of(1);
        public static final int LOOKAHEAD_ITERATIONS = 3;

        public static final int DRIVE_GEAR_TEETH = 110;
        public static final int ENCODER_1_TEETH = 15;
        public static final int ENCODER_2_TEETH = 16;
        public static final Angle MIN_TURN_ANGLE = Rotations.of(-1);
        public static final Angle MAX_TURN_ANGLE = Rotations.of(1);
    }

    public static class IntakeConstants {
        public static final int LEFT_RACK_ID = 0;
        public static final int RIGHT_RACK_ID = 0;
        public static final int LEFT_SPIN_ID = 0;
        public static final int RIGHT_SPIN_ID = 0;

        public static final Slot0Configs RACK_GAINS = new Slot0Configs()
                .withKP(0.0)
                .withKD(0.0)
                .withKA(0.0)
                .withKV(0.0)
                .withKS(0.0);

        public static final Distance STOW_POS = Inches.of(0);
        public static final Distance DEPLOY_POS = Inches.of(10.875);
        public static final Voltage SPIN_VOLTAGE = Volts.of(3);

        public static final LinearVelocity MIN_SWITCH_ROBOT_VELOCITY = MetersPerSecond.of(0.5);

        public static final double VEL_MULTIPLIER = 70.0; // multiplies goal velocity for targetting
        public static final double VEL_POWER = 0.3; // raises goal velocity to power
        public static final LinearVelocity BASE_VEL = InchesPerSecond.of(50); // added to final velocity
    }

    public static class IndexerConstants {}

    public static class ClimberConstants {}

    public static class VisionConstants {
        // Standard deviation baselines for 1 meter distance to single tag
        public static final double LINEAR_STD_DEV_BASELINE = 0.08; // Meters
        public static final double ANGULAR_STD_DEV_BASELINE = 1.0; // Radians

        public static final String[] CAMERA_NAMES = {
            "Arducam_OV9281_FL01", "Arducam_OV9281_FR01",
            "Arducam_OV9281_BL01", "Arducam_OV9281_BR01"
        };

        public static final double MAX_AMBIGUITY = 0.3;

        public static final AprilTagFieldLayout APRIL_TAGS =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Transforms from robot to cameras, (x forward, y left, z up), (roll, pitch,
        // yaw)
        public static final Transform3d[] CAMERA_TRANSFORMS = {
            new Transform3d(new Translation3d(), new Rotation3d()),
            new Transform3d(new Translation3d(), new Rotation3d()),
            new Transform3d(new Translation3d(), new Rotation3d()),
            new Transform3d(new Translation3d(), new Rotation3d())
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
    }

    private Constants() {}
}
