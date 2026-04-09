// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants.ClimbPosition;
import frc.robot.Constants.Dimensions;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AlignToClimb;
import frc.robot.commands.AutoClimb;
import frc.robot.commands.AutoCreator;
import frc.robot.commands.SystemChecks;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.intake.Intakes;
import frc.robot.subsystems.intake.Intakes.IntakesGoal;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.Goal;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretGoal;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.util.FuelSim;
import frc.robot.util.Zones;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Intakes intakes;
    private final Turret turret;
    private final Indexer indexer;
    private final Climber climber;
    private final Superstructure superstructure;
    private final Vision vision;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);
    private final CommandGenericHID toggleSwitches = new CommandGenericHID(1);
    private final CommandGenericHID pushButtons = new CommandGenericHID(2);

    // Commands
    private final TeleopDrive teleopDrive;
    private final Command rumbleLeft = Commands.startEnd(
            () -> controller.setRumble(RumbleType.kLeftRumble, 0.3),
            () -> controller.setRumble(RumbleType.kLeftRumble, 0));
    private final Command rumbleRight = Commands.startEnd(
            () -> controller.setRumble(RumbleType.kRightRumble, 0.3),
            () -> controller.setRumble(RumbleType.kRightRumble, 0));

    // Bindings
    private final Trigger intakeAutoSwitchTrigger = controller.x();
    private final Trigger zeroRightRackTrigger = controller.povRight();
    private final Trigger zeroLeftRackTrigger = controller.povLeft();
    private final Trigger zeroHoodTrigger = controller.povUp();
    private final Trigger switchIntakesTrigger = controller.rightBumper();
    private final Trigger intakePressTrigger = controller.leftBumper();
    private final Trigger collectTrigger = controller.leftTrigger();
    private final Trigger turretModeTrigger = controller.start();
    private final Trigger indexTrigger = controller.back();
    private final Trigger speedUpTrigger = controller.rightTrigger();
    private final Trigger intakeReverseTrigger = controller.b();
    private final Trigger slowDownTrigger;
    //     private final Trigger climbTrigger = controller.y();
    //     private final Trigger extendTrigger = controller.start();
    //     private final Trigger stowTrigger = controller.back();
    private final Trigger manualScoreTrigger = controller.rightTrigger();
    private final Trigger manualPassTrigger = controller.leftTrigger();

    private final Trigger disableIndexer = toggleSwitches.button(1);
    private final Trigger disableVision = toggleSwitches.button(2);
    private final Trigger disableTurret = toggleSwitches.button(3);
    private final Trigger disableLeftIntake = toggleSwitches.button(4);
    private final Trigger disableRightIntake = toggleSwitches.button(5);
    private final Trigger manualOverrideTurret = toggleSwitches.button(8);
    private final Trigger disableWallAvoidance = toggleSwitches.button(7);
    //     private final Trigger flashLEDs = toggleSwitches.button(8);
    private final Trigger hubShiftOverride = toggleSwitches.button(9);
    private final Trigger slowDrive = toggleSwitches.button(10);

    private final Trigger fudgeDown = pushButtons.button(1);
    private final Trigger fudgeUp = pushButtons.button(2);
    private final Trigger zeroHood = pushButtons.button(3);
    private final Trigger zeroLeftIntake = pushButtons.button(4);
    private final Trigger zeroRightIntake = pushButtons.button(5);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;
    private final LoggedNetworkBoolean usePrebuiltAuto;

    public final AutoCreator autoCreator;

    public final SystemChecks systemChecks;

    public FuelSim fuelSim;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        switch (Constants.CURRENT_MODE) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(SwerveConstants.FrontLeft.MODULE_CONSTANTS),
                        new ModuleIOTalonFX(SwerveConstants.FrontRight.MODULE_CONSTANTS),
                        new ModuleIOTalonFX(SwerveConstants.BackLeft.MODULE_CONSTANTS),
                        new ModuleIOTalonFX(SwerveConstants.BackRight.MODULE_CONSTANTS));
                intakes = new Intakes(
                        // new IntakeIO() {},
                        new IntakeIOTalonFX(IntakeConstants.LEFT_RACK_ID, IntakeConstants.LEFT_SPIN_ID),
                        // new IntakeIO() {},
                        new IntakeIOTalonFX(IntakeConstants.RIGHT_RACK_ID, IntakeConstants.RIGHT_SPIN_ID),
                        drive::getChassisSpeeds);
                indexer = new Indexer(new IndexerIOTalonFX());
                // indexer = new Indexer(new IndexerIO() {}, drive::getRotation);
                turret = new Turret(new TurretIOTalonFX(), drive::getPose, drive::getFieldSpeeds);
                // turret = new Turret(new TurretIO() {}, drive::getPose, drive::getFieldSpeeds);
                climber = new Climber(new ClimberIO() {});
                vision = new Vision(
                        drive::addVisionMeasurement,
                        drive::setPose,
                        new VisionIOPhotonVision(VisionConstants.CAMERA_NAMES[0], VisionConstants.CAMERA_TRANSFORMS[0]),
                        new VisionIOPhotonVision(VisionConstants.CAMERA_NAMES[1], VisionConstants.CAMERA_TRANSFORMS[1]),
                        new VisionIOPhotonVision(VisionConstants.CAMERA_NAMES[2], VisionConstants.CAMERA_TRANSFORMS[2]),
                        new VisionIOPhotonVision(VisionConstants.CAMERA_NAMES[3], VisionConstants.CAMERA_TRANSFORMS[3]),
                        new VisionIOPhotonVision(VisionConstants.CAMERA_NAMES[4], VisionConstants.CAMERA_TRANSFORMS[4]),
                        new VisionIOPhotonVision(
                                VisionConstants.CAMERA_NAMES[5], VisionConstants.CAMERA_TRANSFORMS[5]));
                break;

            case SIM:
                configureFuelSim();
                indexer = new Indexer(new IndexerIOSim());
                TurretIOSim turretSim = new TurretIOSim(fuelSim, () -> indexer.getGoal() == IndexerGoal.ACTIVE);
                drive = new Drive(
                        new GyroIO() {},
                        new ModuleIOSim(SwerveConstants.FrontLeft.MODULE_CONSTANTS),
                        new ModuleIOSim(SwerveConstants.FrontRight.MODULE_CONSTANTS),
                        new ModuleIOSim(SwerveConstants.BackLeft.MODULE_CONSTANTS),
                        new ModuleIOSim(SwerveConstants.BackRight.MODULE_CONSTANTS));
                intakes = new Intakes(new IntakeIOSim(), new IntakeIOSim(), drive::getChassisSpeeds);
                turret = new Turret(turretSim, drive::getPose, drive::getFieldSpeeds);
                climber = new Climber(new ClimberIOSim());
                // vision = new Vision(
                //         drive::addVisionMeasurement,
                //         new VisionIOPhotonVisionSim(
                //                 VisionConstants.CAMERA_NAMES[0], VisionConstants.CAMERA_TRANSFORMS[0],
                // drive::getPose),
                //         new VisionIOPhotonVisionSim(
                //                 VisionConstants.CAMERA_NAMES[1], VisionConstants.CAMERA_TRANSFORMS[1],
                // drive::getPose),
                //         new VisionIOPhotonVisionSim(
                //                 VisionConstants.CAMERA_NAMES[2], VisionConstants.CAMERA_TRANSFORMS[2],
                // drive::getPose),
                //         new VisionIOPhotonVisionSim(
                //                 VisionConstants.CAMERA_NAMES[3], VisionConstants.CAMERA_TRANSFORMS[3],
                // drive::getPose),
                //         new VisionIOPhotonVisionSim(
                //                 VisionConstants.CAMERA_NAMES[4], VisionConstants.CAMERA_TRANSFORMS[4],
                // drive::getPose),
                //         new VisionIOPhotonVisionSim(
                //                 VisionConstants.CAMERA_NAMES[5], VisionConstants.CAMERA_TRANSFORMS[5],
                // drive::getPose));
                vision = new Vision(
                        drive::addVisionMeasurement,
                        drive::setPose,
                        new VisionIO() {},
                        new VisionIO() {},
                        new VisionIO() {},
                        new VisionIO() {},
                        new VisionIO() {},
                        new VisionIO() {});
                configureFuelSimRobot(turretSim::canIntake, turretSim::intakeFuel);
                break;

            default:
                // Replayed robot, disable IO implementations
                drive = new Drive(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                intakes = new Intakes(new IntakeIO() {}, new IntakeIO() {}, drive::getChassisSpeeds);
                turret = new Turret(new TurretIO() {}, drive::getPose, drive::getFieldSpeeds);
                indexer = new Indexer(new IndexerIO() {});
                climber = new Climber(new ClimberIO() {});
                vision = new Vision(
                        drive::addVisionMeasurement,
                        drive::setPose,
                        new VisionIO() {},
                        new VisionIO() {},
                        new VisionIO() {},
                        new VisionIO() {},
                        new VisionIO() {},
                        new VisionIO() {});
                break;
        }

        superstructure = new Superstructure(turret, indexer, drive::getPose, drive::getFieldSpeeds);

        // Set up SysId routines
        // autoChooser.addOption(
        //         "Drive Wheel Radius Characterization", DriveCharacterization.wheelRadiusCharacterization(drive));
        // autoChooser.addOption(
        //         "Drive Simple FF Characterization", DriveCharacterization.feedforwardCharacterization(drive));
        // autoChooser.addOption(
        //         "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption(
        //         "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // configure autos
        autoCreator = new AutoCreator();
        AutoBuilder.configure(
                drive::getPose,
                drive::setPose,
                drive::getChassisSpeeds,
                (speeds, feedforwards) -> drive.runVelocity(speeds, feedforwards),
                new PPHolonomicDriveController(
                        AutoConstants.PP_TRANSLATION_CONSTANTS, AutoConstants.PP_ROTATION_CONSTANTS),
                AutoConstants.PP_CONFIG,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                drive);
        PathPlannerLogging.setLogActivePathCallback(
                (path) -> Logger.recordOutput("Odometry/Active Path", path.toArray(Pose2d[]::new)));
        PathPlannerLogging.setLogTargetPoseCallback((target) -> Logger.recordOutput("Odometry/Target Pose", target));

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Prebuilt Autos");
        usePrebuiltAuto = new LoggedNetworkBoolean("Autos/Use Prebuilt Autos", true);

        autoChooser.addDefaultOption("None", Commands.none());

        for (String option : AutoConstants.PREBUILT_AUTOS.keySet()) {
            autoChooser.addOption(
                    option,
                    AutoCreator.buildAuto(
                            drive,
                            vision,
                            intakes,
                            indexer,
                            turret,
                            climber,
                            superstructure,
                            AutoCreator.autoPathsFromString(
                                    AutoConstants.PREBUILT_AUTOS.get(option).substring(2)),
                            false,
                            AutoConstants.PREBUILT_AUTOS.get(option).charAt(0) == 'L'));
        }

        teleopDrive = new TeleopDrive(
                drive, controller, intakes.left.deployedTrigger, intakes.right.deployedTrigger, vision::isEnabled);
        Logger.recordOutput("ZeroedRobotComponents", new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d()});

        Zones.logAllZones();

        SmartDashboard.putData("Align/ClimbFL", new AlignToClimb(ClimbPosition.FRONT_LEFT, drive, vision));
        SmartDashboard.putData("Align/ClimbFR", new AlignToClimb(ClimbPosition.FRONT_RIGHT, drive, vision));
        SmartDashboard.putData("Align/ClimbBL", new AlignToClimb(ClimbPosition.BACK_LEFT, drive, vision));
        SmartDashboard.putData("Align/ClimbBR", new AlignToClimb(ClimbPosition.BACK_RIGHT, drive, vision));
        SmartDashboard.putData(
                "Auto Climb",
                AutoClimb.getAutoClimbCommand(drive, vision, climber)
                        .beforeStarting(superstructure.setGoal(Goal.IDLE)));
        SmartDashboard.putData(
                "Overrides/Manual Pass",
                turret.manualPass()
                        .beforeStarting(indexer.setGoal(IndexerGoal.ACTIVATING))
                        .finallyDo(() -> indexer.stop()));
        SmartDashboard.putData(
                "Overrides/Manual Score",
                turret.manualScore()
                        .beforeStarting(indexer.setGoal(IndexerGoal.ACTIVATING))
                        .finallyDo(() -> indexer.stop()));

        SmartDashboard.putData("Overrides/Slow Drive", teleopDrive.slowDownCommand());

        SmartDashboard.putData("X Wheels", drive.runOnce(drive::stopWithX));

        systemChecks = new SystemChecks(turret, intakes, indexer, climber);

        // Turret turnaround danger zone controller rumble
        turret.turnaroundZoneMaxTrigger.whileTrue(rumbleLeft);
        turret.turnaroundZoneMinTrigger.whileTrue(rumbleRight);

        slowDownTrigger = new Trigger(() -> superstructure.getGoal() == Goal.SCORING);
        slowDownTrigger.whileTrue(teleopDrive.slowDownCommand());

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

        // intakeAutoSwitchTrigger.onTrue(Commands.either(
        //         intakes.setGoal(IntakesGoal.STOW),
        //         intakes.setGoal(IntakesGoal.AUTOSWITCH),
        //         () -> intakes.getGoal() == IntakesGoal.AUTOSWITCH));
        // deployLeftIntakeTrigger.onFalse(intake.setGoal(IntakeGoal.STOW));

        zeroRightRackTrigger.whileTrue(intakes.right.zeroSequence());
        zeroRightRackTrigger.onFalse(intakes.setGoal(IntakesGoal.STOW));

        zeroLeftRackTrigger.whileTrue(intakes.left.zeroSequence());
        zeroLeftRackTrigger.onFalse(intakes.setGoal(IntakesGoal.STOW));

        zeroHoodTrigger.whileTrue(turret.zeroHoodSequence());
        zeroHoodTrigger.onFalse(turret.setGoal(TurretGoal.IDLE));

        switchIntakesTrigger.onTrue(intakes.switchIntakes());
        intakes.switchIntakes().getRequirements().stream().forEach((x) -> System.out.println(x.getName()));
        intakes.deployLeft().getRequirements().stream().forEach((x) -> System.out.println(x.getName()));
        intakes.deployRight().getRequirements().stream().forEach((x) -> System.out.println(x.getName()));
        collectTrigger
                .and(() -> turret.getGoal() != TurretGoal.MANUAL_OVERRIDE)
                .onTrue(superstructure.setGoal(Goal.COLLECTING));
        collectTrigger
                .and(() -> turret.getGoal() != TurretGoal.MANUAL_OVERRIDE)
                .onFalse(superstructure.stopCollecting());
        // collectTrigger.onFalse(superstructure.stopCollecting().onlyIf(() -> superstructure.getGoal() ==
        // Goal.EXPANDED));

        intakePressTrigger.whileTrue(intakes.press());
        intakeReverseTrigger.whileTrue(intakes.reverse());

        speedUpTrigger
                .and(() -> turret.getGoal() != TurretGoal.MANUAL_OVERRIDE)
                .whileTrue(teleopDrive.speedUpCommand());

        // turretTuningTrigger.toggleOnTrue(Commands.startEnd(
        //         turret.setFlywheelSpeed(RPM.of(3000))::initialize, turret.setGoal(TurretGoal.IDLE)::initialize));
        // turretModeTrigger.onTrue(Commands.either(
        //         Commands.either(
        //                 turret.setGoal(TurretGoal.SCORING),
        //                 turret.setGoal(TurretGoal.PASSING),
        //                 superstructure.inAllianceZoneTrigger),
        //         turret.setGoal(TurretGoal.IDLE),
        //         () -> turret.getGoal() == TurretGoal.IDLE));

        turretModeTrigger.onTrue(Commands.either(
                turret.setGoal(TurretGoal.IDLE),
                turret.setGoal(TurretGoal.TUNING),
                () -> turret.getGoal() == TurretGoal.TUNING));

        indexTrigger.onTrue(Commands.either(
                indexer.setGoal(IndexerGoal.IDLE),
                indexer.setGoal(IndexerGoal.ACTIVATING),
                () -> indexer.getGoal() == IndexerGoal.ACTIVE));

        // climbTrigger.toggleOnTrue(
        //         intakes.setGoal(IntakesGoal.STOW).andThen(AutoClimb.getAutoClimbCommand(drive, vision, climber)));
        // extendTrigger.toggleOnTrue(intakes.setGoal(IntakesGoal.STOW).andThen(climber.extend()));
        // stowTrigger.toggleOnTrue(climber.stow());

        manualScoreTrigger
                .and(() -> turret.getGoal() == TurretGoal.MANUAL_OVERRIDE)
                .whileTrue(turret.manualScore()
                        .beforeStarting(indexer.setGoal(IndexerGoal.ACTIVATING).asProxy())
                        .finallyDo(() -> indexer.stop())
                        .withName("Manual Score"));
        manualPassTrigger
                .and(() -> turret.getGoal() == TurretGoal.MANUAL_OVERRIDE)
                .whileTrue(turret.manualPass()
                        .beforeStarting(indexer.setGoal(IndexerGoal.ACTIVATING).asProxy())
                        .finallyDo(() -> indexer.stop())
                        .withName("Manual Pass"));

        disableIndexer.whileTrue(indexer.disable());
        disableVision.whileTrue(vision.disable());
        disableTurret.whileTrue(turret.disable());
        disableLeftIntake.whileTrue(intakes.left.disable());
        disableRightIntake.whileTrue(intakes.right.disable());
        manualOverrideTurret.whileTrue(turret.manualOverride());
        disableWallAvoidance.whileTrue(teleopDrive.disableWallAvoidance());
        // flashLEDs.whileTrue();
        hubShiftOverride.whileTrue(superstructure.enableShiftOverride());
        slowDrive.whileTrue(teleopDrive.slowDownCommand());

        fudgeDown.whileTrue(turret.decreaseFudgeFactor());
        fudgeUp.whileTrue(turret.increaseFudgeFactor());
        zeroHood.whileTrue(turret.zeroHoodSequence());
        zeroLeftIntake.whileTrue(intakes.left.zeroSequence());
        zeroLeftIntake.onFalse(intakes.left.stow());
        zeroRightIntake.whileTrue(intakes.right.zeroSequence());
        zeroRightIntake.onFalse(intakes.right.stow());
    }

    private void configureFuelSim() {
        fuelSim = new FuelSim("/AdvantageKit");
        fuelSim.spawnStartingFuel();
        fuelSim.setSubticks(1);
        fuelSim.setLoggingFrequency(20);

        fuelSim.start();
        SmartDashboard.putData(Commands.runOnce(() -> {
                    fuelSim.clearFuel();
                    fuelSim.spawnStartingFuel();
                })
                .withName("Reset Fuel")
                .ignoringDisable(true));
    }

    private void configureFuelSimRobot(BooleanSupplier ableToIntake, Runnable intakeCallback) {
        fuelSim.registerRobot(
                Dimensions.FULL_WIDTH.in(Meters),
                Dimensions.FULL_LENGTH.in(Meters),
                Dimensions.BUMPER_HEIGHT.in(Meters),
                drive::getPose,
                drive::getFieldSpeeds);
        fuelSim.registerIntake(
                -Dimensions.FULL_LENGTH.div(2).in(Meters),
                Dimensions.FULL_LENGTH.div(2).in(Meters),
                -Dimensions.FULL_WIDTH.div(2).plus(Inches.of(7)).in(Meters),
                -Dimensions.FULL_WIDTH.div(2).in(Meters),
                intakes.right.deployedTrigger.and(ableToIntake),
                intakeCallback);
        fuelSim.registerIntake(
                -Dimensions.FULL_LENGTH.div(2).in(Meters),
                Dimensions.FULL_LENGTH.div(2).in(Meters),
                Dimensions.FULL_WIDTH.div(2).in(Meters),
                Dimensions.FULL_WIDTH.div(2).plus(Inches.of(7)).in(Meters),
                intakes.left.deployedTrigger.and(ableToIntake),
                intakeCallback);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        if (!usePrebuiltAuto.get()) {
            return autoCreator.buildAuto(drive, vision, intakes, indexer, turret, climber, superstructure);
        }

        return autoChooser.get();
    }
}
