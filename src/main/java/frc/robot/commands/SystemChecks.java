package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot.Constants.TurretConstants.MIN_HOOD_ANGLE;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.Indexer.IndexerGoal;
import frc.robot.subsystems.intake.Intakes;
import frc.robot.subsystems.intake.Intakes.IntakesGoal;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.Turret.TurretGoal;

/** Class to run various system checks on the robot */
public class SystemChecks {
    private final Turret turret;
    private final Intakes intakes;
    private final Indexer indexer;

    public SystemChecks(Turret turret, Intakes intakes, Indexer indexer) {
        this.turret = turret;
        this.intakes = intakes;
        this.indexer = indexer;

        SmartDashboard.putData("System Checks/Turret", turret());
        SmartDashboard.putData("System Checks/TurretTurn", turretTurn());
        SmartDashboard.putData("System Checks/TurretHood", turretHood());
        SmartDashboard.putData("System Checks/TurretFlywheels", turretFlywheels());
        SmartDashboard.putData("System Checks/Intakes", intakes());
        SmartDashboard.putData("System Checks/LeftIntake", leftIntake());
        SmartDashboard.putData("System Checks/RightIntake", rightIntake());
        SmartDashboard.putData("System Checks/Indexer", indexer());
    }

    /** Check turret turn, then hood, then flywheels */
    public Command turret() {
        return Commands.sequence(turretTurn(), turretHood(), turretFlywheels());
    }

    /** Turns turret to 0, then 45 degrees, then -45 degrees, then back to 0 */
    public Command turretTurn() {
        return Commands.sequence(
                turret.setTurnPosition(Degrees.of(0)),
                Commands.waitSeconds(0.5),
                turret.setTurnPosition(Degrees.of(180)),
                Commands.waitSeconds(0.5),
                turret.setTurnPosition(Degrees.of(-180)),
                Commands.waitSeconds(0.5),
                turret.setTurnPosition(Degrees.of(0)));
    }

    /** Zeroes hood, then moves to 30 degrees, then back to min */
    public Command turretHood() {
        return Commands.sequence(
                turret.zeroHoodSequence(),
                Commands.waitSeconds(0.5),
                turret.setHoodPosition(Degrees.of(30)),
                Commands.waitSeconds(0.5),
                turret.setHoodPosition(MIN_HOOD_ANGLE));
    }

    /** Spins flywheels up to 2500 RPM for 3 seconds, then turns them off */
    public Command turretFlywheels() {
        return Commands.sequence(
                turret.setFlywheelSpeed(RPM.of(2500)), Commands.waitSeconds(3), turret.setGoal(TurretGoal.IDLE));
    }

    /** Zeroes both intakes then tests left then right */
    public Command intakes() {
        return Commands.sequence(
                intakes.left.zeroSequence(),
                intakes.right.zeroSequence(),
                leftIntake(),
                rightIntake(),
                Commands.waitSeconds(0.5),
                intakes.setGoal(IntakesGoal.IDLE));
    }

    /** Deploy left intake, then bring it back in */
    public Command leftIntake() {
        return Commands.sequence(intakes.deployLeft(), Commands.waitSeconds(1), intakes.setGoal(IntakesGoal.STOW));
    }

    /** Deploy right intake, then bring it back in */
    public Command rightIntake() {
        return Commands.sequence(intakes.deployRight(), Commands.waitSeconds(1), intakes.setGoal(IntakesGoal.STOW));
    }

    /** Activate indexer for 3 seconds */
    public Command indexer() {
        return Commands.sequence(
                indexer.setGoal(IndexerGoal.ACTIVATING), Commands.waitSeconds(3), indexer.setGoal(IndexerGoal.IDLE));
    }
}
