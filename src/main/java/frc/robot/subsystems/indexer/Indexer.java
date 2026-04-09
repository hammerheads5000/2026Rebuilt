package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IndexerConstants.FEED_STALL_ANGULAR_VELOCITY;
import static frc.robot.Constants.IndexerConstants.FEED_STALL_CURRENT;
import static frc.robot.Constants.IndexerConstants.FEED_VOLTAGE;
import static frc.robot.Constants.IndexerConstants.HOOK_STALL_ANGULAR_VELOCITY;
import static frc.robot.Constants.IndexerConstants.HOOK_STALL_CURRENT;
import static frc.robot.Constants.IndexerConstants.SPIN_RAMP;
import static frc.robot.Constants.IndexerConstants.SPIN_VOLTAGE;
import static frc.robot.Constants.IndexerConstants.UNJAM_FEED_VOLTAGE;
import static frc.robot.Constants.IndexerConstants.UNJAM_SPIN_VOLTAGE;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.VirtualPD;
import java.util.Set;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private final LoggedTunableNumber spinVoltageTunable =
            new LoggedTunableNumber("Indexer/Spin Voltage", SPIN_VOLTAGE.in(Volts));
    private final LoggedTunableNumber feedVoltageTunable =
            new LoggedTunableNumber("Indexer/Feed Voltage", FEED_VOLTAGE.in(Volts));

    private final SlewRateLimiter spinVoltageLimiter =
            new SlewRateLimiter(spinVoltageTunable.get() / SPIN_RAMP.in(Seconds));

    @AutoLogOutput
    private final Trigger hookStallTrigger = new Trigger(
                    () -> inputs.spinCurrent.in(Amps) >= HOOK_STALL_CURRENT.in(Amps)
                            && inputs.spinVelocity.abs(RPM) <= HOOK_STALL_ANGULAR_VELOCITY.in(RPM))
            .debounce(0.2);

    @AutoLogOutput
    private final Trigger feedStallTrigger = new Trigger(
                    () -> inputs.feedCurrent.in(Amps) >= FEED_STALL_CURRENT.in(Amps)
                            && inputs.feedVelocity.abs(RPM) <= FEED_STALL_ANGULAR_VELOCITY.in(RPM))
            .debounce(0.2);

    private final IndexerVisualizer visualizer;

    private final Alert feedDisconnectedAlert = new Alert("Indexer Feed Motor Disconnected", AlertType.kError);
    private final Alert hookDisconnectedAlert = new Alert("Indexer Hook Motor Disconnected", AlertType.kError);
    private final Alert disabledAlert = new Alert("Indexer Disabled", AlertType.kWarning);

    @AutoLogOutput
    private IndexerGoal goal = IndexerGoal.IDLE;

    public Indexer(IndexerIO io) {
        this.io = io;
        this.visualizer = new IndexerVisualizer();

        VirtualPD.registerMotor(() -> inputs.spinSupplyCurrent, "Indexer");
        VirtualPD.registerMotor(() -> inputs.feedSupplyCurrent, "Indexer");

        hookStallTrigger
                .or(feedStallTrigger)
                .and(() -> this.goal == IndexerGoal.ACTIVE)
                .onTrue(setGoal(IndexerGoal.UNJAMMING));

        SmartDashboard.putData("Overrides/Indexer", disable());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
        Logger.recordOutput(
                "Indexer/Current Command",
                this.getCurrentCommand() == null
                        ? "None"
                        : this.getCurrentCommand().getName());

        if ((spinVoltageTunable.hasChanged(hashCode()) || feedVoltageTunable.hasChanged(hashCode()))
                && (goal == IndexerGoal.ACTIVE || goal == IndexerGoal.ACTIVATING)) {
            io.setSpinOutput(Volts.of(spinVoltageTunable.get()));
            io.setFeedOutput(Volts.of(feedVoltageTunable.get()));
        }

        visualizer.update(inputs.spinVelocity);

        feedDisconnectedAlert.set(!inputs.feedMotorConnected && Constants.CURRENT_MODE != Constants.Mode.SIM);
        hookDisconnectedAlert.set(!inputs.spinMotorConnected && Constants.CURRENT_MODE != Constants.Mode.SIM);
    }

    public Command setGoal(IndexerGoal newGoal) {
        return Commands.defer(
                        () -> {
                            if (this.goal != newGoal) {
                                return Commands.none();
                            }
                            Command toSchedule = Commands.none();

                            switch (newGoal) {
                                case ACTIVATING:
                                    toSchedule = activate();
                                    break;
                                case ACTIVE:
                                    break;
                                case DISABLED:
                                    toSchedule = this.runOnce(this::stop);
                                    break;
                                case IDLE:
                                    toSchedule = this.runOnce(this::stop);
                                    break;
                                case UNJAMMING:
                                    toSchedule = unjam();
                                    break;
                            }
                            this.goal = newGoal;
                            return new ScheduleCommand(toSchedule);
                        },
                        Set.of(this))
                .onlyIf(() -> this.goal != IndexerGoal.DISABLED);
    }

    public Command disable() {
        return this.runOnce(() -> {
                    goal = IndexerGoal.DISABLED;
                    io.stopFeed();
                    io.stopSpin();
                    disabledAlert.set(true);
                })
                .andThen(Commands.idle())
                .finallyDo(() -> {
                    goal = IndexerGoal.IDLE;
                    disabledAlert.set(false);
                })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .ignoringDisable(true)
                .withName("Disable Indexer");
    }

    public IndexerGoal getGoal() {
        return goal;
    }

    private Command activate() {
        return Commands.sequence(
                        this.runOnce(() -> io.setFeedOutput(UNJAM_FEED_VOLTAGE)),
                        Commands.waitSeconds(0.1),
                        this.runOnce(() -> io.setFeedOutput(Volts.of(feedVoltageTunable.get()))),
                        Commands.waitSeconds(0.2),
                        this.startRun(
                                        () -> spinVoltageLimiter.reset(0),
                                        () -> io.setSpinOutput(
                                                Volts.of(spinVoltageLimiter.calculate(spinVoltageTunable.get()))))
                                .withTimeout(SPIN_RAMP))
                .finallyDo((interrupted) -> {
                    if (interrupted) {
                        io.stopFeed();
                        io.stopSpin();
                    }
                    goal = IndexerGoal.ACTIVE;
                })
                // .andThen(Commands.waitUntil(() -> inputs.feedVelocity.abs(RPM) >= FEED_THRESHOLD.in(RPM)))
                // .andThen(this.runOnce(() -> io.setSpinOutput(Volts.of(spinVoltage.get()))))
                .withName("Indexer Activate");
    }

    private Command unjam() {
        return Commands.sequence(
                        this.runOnce(() -> {
                            io.setFeedOutput(UNJAM_FEED_VOLTAGE);
                            io.setSpinOutput(UNJAM_SPIN_VOLTAGE);
                        }),
                        Commands.waitSeconds(0.2),
                        Commands.runOnce(() -> {
                            io.setFeedOutput(Volts.of(feedVoltageTunable.get()));
                            io.setSpinOutput(Volts.of(0));
                        }),
                        Commands.waitSeconds(0.1),
                        Commands.runOnce(() -> io.setSpinOutput(Volts.of(spinVoltageTunable.get()))))
                .finallyDo((interrupted) -> {
                    if (interrupted) {
                        io.stopFeed();
                        io.stopSpin();
                    } else {
                        goal = IndexerGoal.ACTIVE;
                    }
                })
                .withName("Indexer Unjam");
    }

    public void stop() {
        io.stopSpin();
        io.stopFeed();
        this.goal = IndexerGoal.IDLE;
    }

    public enum IndexerGoal {
        ACTIVATING,
        ACTIVE,
        UNJAMMING,
        IDLE,
        DISABLED // manual override
    }
}
