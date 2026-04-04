package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.DEPLOY_POS;
import static frc.robot.Constants.IntakeConstants.DEPLOY_TOLERANCE;
import static frc.robot.Constants.IntakeConstants.LEFT_RACK_GAINS;
import static frc.robot.Constants.IntakeConstants.LEFT_ROTOR_TO_PINION_RATIO;
import static frc.robot.Constants.IntakeConstants.MAX_POS;
import static frc.robot.Constants.IntakeConstants.PRESS_IN_TIME;
import static frc.robot.Constants.IntakeConstants.PRESS_IN_VOLTAGE;
import static frc.robot.Constants.IntakeConstants.PRESS_OUT_TIME;
import static frc.robot.Constants.IntakeConstants.PRESS_OUT_VOLTAGE;
import static frc.robot.Constants.IntakeConstants.PRESS_STOP_SPIN;
import static frc.robot.Constants.IntakeConstants.RACK_MOTION_MAGIC;
import static frc.robot.Constants.IntakeConstants.RACK_STALL_CURRENT;
import static frc.robot.Constants.IntakeConstants.RACK_STALL_VEL;
import static frc.robot.Constants.IntakeConstants.REVERSE_SPIN_VOLTAGE;
import static frc.robot.Constants.IntakeConstants.RIGHT_ROTOR_TO_PINION_RATIO;
import static frc.robot.Constants.IntakeConstants.SPIN_STALL_ANGULAR_VELOCITY;
import static frc.robot.Constants.IntakeConstants.SPIN_STALL_CURRENT;
import static frc.robot.Constants.IntakeConstants.SPIN_VOLTAGE;
import static frc.robot.Constants.IntakeConstants.STOW_POS;
import static frc.robot.Constants.IntakeConstants.STOW_TOLERANCE;
import static frc.robot.Constants.IntakeConstants.UNJAM_SPIN_VOLTAGE;
import static frc.robot.Constants.IntakeConstants.ZEROING_VOLTAGE;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.intake.Intakes.IntakeSide;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualPD;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final IntakeSide side;
    private final String name;

    @AutoLogOutput(key = "Intakes/{name}/Goal")
    private IntakeGoal goal = IntakeGoal.STOWED;

    @AutoLogOutput(key = "Intakes/{name}/Rack Stalled")
    public final Trigger rackStallTrigger;

    @AutoLogOutput(key = "Intakes/{name}/Spin Stalled")
    public final Trigger spinStallTrigger;

    @AutoLogOutput(key = "Intakes/{name}/Stowed")
    public final Trigger stowedTrigger;

    @AutoLogOutput(key = "Intakes/{name}/Deployed")
    public final Trigger deployedTrigger;

    private final LoggedTunableNumber rackKP;
    private final LoggedTunableNumber rackKD;
    private final LoggedTunableNumber rackKV;
    private final LoggedTunableNumber rackKA;
    private final LoggedTunableNumber rackKS;
    private final LoggedTunableNumber rackMaxVel =
            new LoggedTunableNumber("Intakes/maxVelRotPerSec", RACK_MOTION_MAGIC.MotionMagicCruiseVelocity);
    private final LoggedTunableNumber rackMaxAcc =
            new LoggedTunableNumber("Intakes/maxAccRotPerSecPerSec", RACK_MOTION_MAGIC.MotionMagicAcceleration);
    private final LoggedTunableNumber spinVoltage =
            new LoggedTunableNumber("Intakes/Spin Voltage", SPIN_VOLTAGE.in(Volts));
    private final LoggedTunableNumber reverseSpinVoltage =
            new LoggedTunableNumber("Intakes/Reverse Spin Voltage", REVERSE_SPIN_VOLTAGE.in(Volts));
    private final LoggedTunableNumber deployPos =
            new LoggedTunableNumber("Intakes/DeployPosInches", DEPLOY_POS.in(Inches));
    private final LoggedTunableNumber pressInVoltage =
            new LoggedTunableNumber("Intakes/Press In Voltage", PRESS_IN_VOLTAGE.in(Volts));
    private final LoggedTunableNumber pressOutVoltage =
            new LoggedTunableNumber("Intakes/Press Out Voltage", PRESS_OUT_VOLTAGE.in(Volts));
    private final LoggedTunableNumber pressInTime =
            new LoggedTunableNumber("Intakes/Press In Time", PRESS_IN_TIME.in(Seconds));
    private final LoggedTunableNumber pressOutTime =
            new LoggedTunableNumber("Intakes/Press Out Time", PRESS_OUT_TIME.in(Seconds));

    private final Alert rackDisconnectedAlert;
    private final Alert spinDisconnectedAlert;
    private final Alert disabledAlert;

    public Intake(IntakeIO io, IntakeSide side) {
        this.io = io;
        this.side = side;
        this.name = side.toString();

        VirtualPD.registerMotor(() -> inputs.spinSupplyCurrent, "Intakes/" + name);
        VirtualPD.registerMotor(() -> inputs.rackSupplyCurrent, "Intakes/" + name);

        double multiplier = side == IntakeSide.Right ? RIGHT_ROTOR_TO_PINION_RATIO / LEFT_ROTOR_TO_PINION_RATIO : 1;

        rackKP = new LoggedTunableNumber("Intakes/" + name + "/kP", LEFT_RACK_GAINS.kP * multiplier);
        rackKD = new LoggedTunableNumber("Intakes/" + name + "/kD", LEFT_RACK_GAINS.kD * multiplier);
        rackKV = new LoggedTunableNumber("Intakes/" + name + "/kV", LEFT_RACK_GAINS.kV * multiplier);
        rackKA = new LoggedTunableNumber("Intakes/" + name + "/kA", LEFT_RACK_GAINS.kA * multiplier);
        rackKS = new LoggedTunableNumber("Intakes/" + name + "/kS", LEFT_RACK_GAINS.kS);

        rackDisconnectedAlert = new Alert(name + " Intake Rack Motor Disconnected!", AlertType.kError);
        spinDisconnectedAlert = new Alert(name + " Intake Spin Motor Disconnected!", AlertType.kError);
        disabledAlert = new Alert(name + " Disabled", AlertType.kWarning);

        spinStallTrigger = new Trigger(this::spinStalled).debounce(0.5);
        rackStallTrigger = new Trigger(this::rackStalled).debounce(0.1);
        deployedTrigger = new Trigger(this::deployed)
                .and(() -> this.goal == IntakeGoal.DEPLOYING || this.goal == IntakeGoal.DEPLOYED)
                .debounce(0.05);
        stowedTrigger = new Trigger(this::stowed)
                .and(() -> this.goal == IntakeGoal.STOWING || this.goal == IntakeGoal.STOWED)
                .debounce(0.05);

        spinStallTrigger
                .or(rackStallTrigger)
                .and(() -> this.goal != IntakeGoal.ZEROING && this.goal != IntakeGoal.IDLE)
                .onTrue(unjam());
        deployedTrigger.onTrue(setGoal(IntakeGoal.DEPLOYED));
        stowedTrigger.and(() -> this.goal == IntakeGoal.STOWING).onTrue(setGoal(IntakeGoal.STOWED));

        SmartDashboard.putData("Overrides/" + side.name() + " Intake", disable());
    }

    private boolean spinStalled() {
        return inputs.spinCurrent.abs(Amps) >= SPIN_STALL_CURRENT.in(Amps)
                && inputs.spinVelocity.abs(RadiansPerSecond) < SPIN_STALL_ANGULAR_VELOCITY.in(RadiansPerSecond);
    }

    private boolean rackStalled() {
        return inputs.rackCurrent.abs(Amps) >= RACK_STALL_CURRENT.in(Amps)
                && inputs.rackVelocity.abs(MetersPerSecond) < RACK_STALL_VEL.in(MetersPerSecond);
    }

    private boolean deployed() {
        return inputs.rackPosition.isNear(Inches.of(deployPos.get()), DEPLOY_TOLERANCE);
    }

    private boolean stowed() {
        return inputs.rackPosition.isNear(STOW_POS, STOW_TOLERANCE);
    }

    public Command zeroSequence() {
        return Commands.sequence(
                this.setGoal(IntakeGoal.ZEROING),
                this.runOnce(() -> io.setRackOutput(ZEROING_VOLTAGE)),
                Commands.waitSeconds(0.1),
                Commands.waitUntil(rackStallTrigger::getAsBoolean),
                this.runOnce(io::stopRack),
                Commands.waitSeconds(0.1),
                this.runOnce(() -> {
                    io.zeroPosition();
                    io.setRackPosition(STOW_POS);
                }),
                this.setGoal(IntakeGoal.STOWED));
    }

    public Command deploy() {
        return this.setGoal(IntakeGoal.DEPLOYING).withName("Deploy " + name);
    }

    public Command stow() {
        return this.setGoal(IntakeGoal.STOWING).withName("Stow " + name);
    }

    public Command off() {
        return this.setGoal(IntakeGoal.IDLE);
    }

    private Command unjam() {
        return Commands.select(
                Map.of(
                        IntakeGoal.STOWING, // extend intake and reverse rollers
                        Commands.sequence(
                                Commands.runOnce(() -> io.setRackPosition(Inches.of(deployPos.get()))),
                                Commands.waitUntil(
                                        rackStallTrigger.or(spinStallTrigger).negate()),
                                Commands.runOnce(() -> io.setRackPosition(STOW_POS))),
                        IntakeGoal.DEPLOYED, // reverse rollers
                        Commands.none(),
                        // Commands.sequence(
                        //         Commands.runOnce(() -> io.setSpinOutput(UNJAM_SPIN_VOLTAGE.unaryMinus())),
                        //         Commands.waitSeconds(0.1),
                        //         Commands.runOnce(() -> io.setSpinOutput(Volts.of(spinVoltage.get())))),
                        IntakeGoal.STOWED, // unreverse rollers
                        Commands.sequence(
                                Commands.runOnce(() -> io.setSpinOutput(UNJAM_SPIN_VOLTAGE)),
                                Commands.waitUntil(spinStallTrigger.negate()),
                                Commands.runOnce(() -> io.stopSpin())),
                        IntakeGoal.DEPLOYING, // stow intake
                        Commands.sequence(
                                Commands.runOnce(() -> io.setRackPosition(STOW_POS)),
                                Commands.waitUntil(rackStallTrigger.negate()),
                                Commands.runOnce(() -> io.setRackPosition(Inches.of(deployPos.get())))),
                        IntakeGoal.IDLE,
                        Commands.none(),
                        IntakeGoal.PRESSING,
                        Commands.none(),
                        IntakeGoal.DISABLED,
                        Commands.none()),
                () -> this.goal);
    }

    private Command setGoal(IntakeGoal goal) {
        return Commands.runOnce(() -> {
                    this.goal = goal;
                    switch (goal) {
                        case DEPLOYED:
                            // io.stopRack();
                            break;
                        case DEPLOYING:
                            io.setSpinOutput(Volts.of(spinVoltage.get()));
                            io.setRackPosition(Inches.of(deployPos.get()));
                            break;
                        case IDLE:
                            io.stopRack();
                            io.stopSpin();
                            break;
                        case STOWED:
                            io.stopSpin();
                            break;
                        case STOWING:
                            io.setRackPosition(STOW_POS);
                            break;
                        case PRESSING:
                            break;
                        case ZEROING:
                            break;
                        case DISABLED:
                            io.stopRack();
                            io.stopSpin();
                            break;
                    }
                })
                .onlyIf(() -> this.goal != IntakeGoal.DISABLED)
                .withName("Set goal");
    }

    // slowly move intake in and out to push balls into indexer
    public Command press() {
        return Commands.repeatingSequence(
                        this.runOnce(() -> io.setRackOutput(Volts.of(pressInVoltage.get()))),
                        Commands.waitSeconds(pressInTime.get()),
                        this.runOnce(() -> io.setRackOutput(Volts.of(pressOutVoltage.get()))),
                        Commands.waitSeconds(pressOutTime.get()))
                .alongWith(
                        Commands.waitUntil(() -> inputs.rackPosition.lte(PRESS_STOP_SPIN)),
                        Commands.runOnce(io::stopSpin))
                .until(this::stowed)
                .finallyDo(() -> {
                    this.goal = IntakeGoal.DEPLOYING;
                    io.setRackPosition(Inches.of(deployPos.get()));
                    io.setSpinOutput(Volts.of(spinVoltage.get()));
                })
                .withName("Press intake " + side.name());
    }

    public Command reverse() {
        return this.startEnd(
                        () -> {
                            io.setSpinOutput(Volts.of(reverseSpinVoltage.get()));
                            io.setRackPosition(MAX_POS);
                        },
                        () -> {
                            if (goal == IntakeGoal.DEPLOYED) {
                                io.setSpinOutput(Volts.of(spinVoltage.get()));
                                io.setRackPosition(DEPLOY_POS);
                            } else {
                                io.stopSpin();
                            }
                        })
                .onlyWhile(() -> goal == IntakeGoal.DEPLOYED)
                .withName("Reverse intake " + side.name());
    }

    public Command disable() {
        return this.runOnce(() -> {
                    goal = IntakeGoal.DISABLED;
                    io.stopRack();
                    io.stopSpin();
                    disabledAlert.set(true);
                })
                .andThen(Commands.idle())
                .finallyDo(() -> {
                    goal = IntakeGoal.IDLE;
                    disabledAlert.set(false);
                })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .ignoringDisable(true)
                .withName("Disable Intake " + side.name());
    }

    public IntakeGoal getGoal() {
        return goal;
    }

    private void updateTunables() {
        if (rackKP.hasChanged(hashCode())
                || rackKD.hasChanged(hashCode())
                || rackKV.hasChanged(hashCode())
                || rackKA.hasChanged(hashCode())
                || rackKS.hasChanged(hashCode())
                || rackMaxVel.hasChanged(hashCode())
                || rackMaxAcc.hasChanged(hashCode())) {
            io.setRackPID(
                    rackKP.get(),
                    rackKD.get(),
                    rackKV.get(),
                    rackKA.get(),
                    rackKS.get(),
                    rackMaxVel.get(),
                    rackMaxAcc.get());
        }

        if (spinVoltage.hasChanged(hashCode())) {
            if (goal == IntakeGoal.DEPLOYED || goal == IntakeGoal.DEPLOYING) {
                io.setSpinOutput(Volts.of(spinVoltage.get()));
            }
        }

        // if (reverseSpinVoltage.hasChanged(hashCode())) {
        //     if (goal == IntakeGoal.STOWED || goal == IntakeGoal.STOWING) {
        //         io.setSpinOutput(Volts.of(reverseSpinVoltage.get()));
        //     }
        // }
    }

    public Distance getPosition() {
        return inputs.rackPosition;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intakes/" + name, inputs);
        Logger.recordOutput(
                "Intakes/" + name + "/Current Command",
                this.getCurrentCommand() == null
                        ? "None"
                        : this.getCurrentCommand().getName());
        updateTunables();

        rackDisconnectedAlert.set(!inputs.rackMotorConnected && Constants.CURRENT_MODE != Mode.SIM);
        spinDisconnectedAlert.set(!inputs.spinMotorConnected && Constants.CURRENT_MODE != Mode.SIM);
    }

    public enum IntakeGoal {
        DEPLOYED,
        DEPLOYING,
        STOWED,
        STOWING,
        PRESSING,
        ZEROING,
        IDLE,
        DISABLED
    }
}
