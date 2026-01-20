// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.DriveCharacterization;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Intake intake;
    private final Turret turret;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Commands
    private final TeleopDrive teleopDrive;

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    // Maple sim
    private final SwerveDriveSimulation driveSimulation;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                driveSimulation = null;

                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(SwerveConstants.FrontLeft.MODULE_CONSTANTS),
                        new ModuleIOTalonFX(SwerveConstants.FrontRight.MODULE_CONSTANTS),
                        new ModuleIOTalonFX(SwerveConstants.BackLeft.MODULE_CONSTANTS),
                        new ModuleIOTalonFX(SwerveConstants.BackRight.MODULE_CONSTANTS));
                intake = new Intake(
                        new IntakeIOTalonFX(IntakeConstants.LEFT_RACK_ID, IntakeConstants.LEFT_SPIN_ID),
                        new IntakeIOTalonFX(IntakeConstants.RIGHT_RACK_ID, IntakeConstants.RIGHT_SPIN_ID),
                        drive::getChassisSpeeds);
                turret = new Turret(new TurretIOTalonFX(), drive::getPose, drive::getFieldSpeeds);
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                SimulatedArena.overrideSimulationTimings(
                        Seconds.of(Robot.defaultPeriodSecs), SwerveConstants.SIM_TICKS_PER_PERIOD);
                driveSimulation = new SwerveDriveSimulation(
                        DriveTrainSimulationConfig.Default()
                                .withRobotMass(SwerveConstants.ROBOT_MASS)
                                .withBumperSize(
                                        Dimensions.FRAME_LENGTH.plus(Dimensions.BUMPER_THICKNESS.times(2)),
                                        Dimensions.FRAME_WIDTH.plus(Dimensions.BUMPER_THICKNESS.times(2)))
                                .withTrackLengthTrackWidth(
                                        SwerveConstants.MODULE_DISTANCE_X, SwerveConstants.MODULE_DISTANCE_Y)
                                .withSwerveModule(new SwerveModuleSimulationConfig(
                                        SwerveConstants.DRIVE_MOTOR_MODEL,
                                        SwerveConstants.STEER_MOTOR_MODEL,
                                        SwerveConstants.DRIVE_GEAR_RATIO,
                                        SwerveConstants.STEER_GEAR_RATIO,
                                        SwerveConstants.DRIVE_FRICTION_VOLTAGE,
                                        SwerveConstants.STEER_FRICTION_VOLTAGE,
                                        SwerveConstants.WHEEL_RADIUS,
                                        SwerveConstants.STEER_INERTIA,
                                        SwerveConstants.WHEEL_COEFFICIENT_OF_FRICTION))
                                .withGyro(SwerveConstants.GYRO_SIMULATION),
                        new Pose2d(3, 4, Rotation2d.kZero));
                SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

                drive = new Drive(
                        new GyroIOSim(SwerveConstants.GYRO_SIMULATION.get()),
                        new ModuleIOSim(driveSimulation.getModules()[0]),
                        new ModuleIOSim(driveSimulation.getModules()[1]),
                        new ModuleIOSim(driveSimulation.getModules()[2]),
                        new ModuleIOSim(driveSimulation.getModules()[3]));
                intake = new Intake(new IntakeIOSim(), new IntakeIOSim(), drive::getChassisSpeeds);
                turret = new Turret(new TurretIOSim(), drive::getPose, drive::getFieldSpeeds);

                driveSimulation.setSimulationWorldPose(
                        FieldMirroringUtils.toCurrentAlliancePose(new Pose2d(3, 4, Rotation2d.kZero)));
                drive.setPose(FieldMirroringUtils.toCurrentAlliancePose(new Pose2d(3, 4, Rotation2d.kZero)));
                SimulatedArena.getInstance().resetFieldForAuto();
                break;

            default:
                // Replayed robot, disable IO implementations
                driveSimulation = null;

                drive = new Drive(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                intake = new Intake(new IntakeIO() {}, new IntakeIO() {}, drive::getChassisSpeeds);
                turret = new Turret(new TurretIO() {}, drive::getPose, drive::getFieldSpeeds);
                break;
        }

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices");

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCharacterization.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCharacterization.feedforwardCharacterization(drive));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        teleopDrive = new TeleopDrive(drive, controller);
        Logger.recordOutput("ZeroedRobotComponents", new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d()});
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(teleopDrive);
    }

    public void updateFieldSimAndDisplay() {
        if (driveSimulation == null) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput("FieldSimulation/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
