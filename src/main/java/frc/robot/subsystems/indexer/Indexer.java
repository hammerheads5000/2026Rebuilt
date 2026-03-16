package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IndexerConstants.FEED_VOLTAGE;
import static frc.robot.Constants.IndexerConstants.SPIN_STALL_ANGULAR_VELOCITY;
import static frc.robot.Constants.IndexerConstants.SPIN_STALL_CURRENT;
import static frc.robot.Constants.IndexerConstants.SPIN_VOLTAGE;
import static frc.robot.Constants.IndexerConstants.UNJAM_FEED_VOLTAGE;
import static frc.robot.Constants.IndexerConstants.UNJAM_SPIN_VOLTAGE;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    private final LoggedTunableNumber spinVoltage =
            new LoggedTunableNumber("Indexer/Spin Voltage", SPIN_VOLTAGE.in(Volts));
    private final LoggedTunableNumber feedVoltage =
            new LoggedTunableNumber("Indexer/Feed Voltage", FEED_VOLTAGE.in(Volts));

    private final Trigger spinStallTrigger = new Trigger(
                    () -> inputs.spinCurrent.abs(Amps) >= SPIN_STALL_CURRENT.in(Amps)
                            && inputs.spinVelocity.abs(RPM) <= SPIN_STALL_ANGULAR_VELOCITY.in(RPM))
            .debounce(0.2);

    private final IndexerVisualizer visualizer;

    private final Alert feedDisconnectedAlert = new Alert("Indexer Feed Motor Disconnected", AlertType.kError);
    private final Alert hookDisconnectedAlert = new Alert("Indexer Hook Motor Disconnected", AlertType.kError);

    @AutoLogOutput
    private IndexerGoal goal = IndexerGoal.IDLE;

    public Indexer(IndexerIO io) {
        this.io = io;
        this.visualizer = new IndexerVisualizer();

        VirtualPD.registerMotor(() -> inputs.spinSupplyCurrent, "Indexer");
        VirtualPD.registerMotor(() -> inputs.feedSupplyCurrent, "Indexer");

        spinStallTrigger.and(() -> this.goal == IndexerGoal.ACTIVE).onTrue(unjam());

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

        if ((spinVoltage.hasChanged(hashCode()) || feedVoltage.hasChanged(hashCode())) && goal == IndexerGoal.ACTIVE) {
            io.setSpinOutput(Volts.of(spinVoltage.get()));
            io.setFeedOutput(Volts.of(feedVoltage.get()));
        }

        visualizer.update(inputs.spinVelocity);

        feedDisconnectedAlert.set(!inputs.feedMotorConnected && Constants.CURRENT_MODE != Constants.Mode.SIM);
        hookDisconnectedAlert.set(!inputs.spinMotorConnected && Constants.CURRENT_MODE != Constants.Mode.SIM);
    }

    public Command setGoal(IndexerGoal goal) {
        return Commands.defer(
                        () -> {
                            Command toSchedule = Commands.none();

                            if (goal == IndexerGoal.ACTIVE && this.goal != IndexerGoal.ACTIVE) {
                                // if (this.goal != IndexerGoal.ACTIVE) {
                                toSchedule = activate();
                                // } else {
                                //     toSchedule = this.runOnce(() -> {
                                //         io.setSpinOutput(Volts.of(spinVoltage.get()));
                                //         io.setFeedOutput(Volts.of(feedVoltage.get()));
                                //     });
                                // }
                            } else if (goal == IndexerGoal.IDLE) {
                                toSchedule = this.runOnce(this::stop);
                            }
                            this.goal = goal;
                            return toSchedule;
                        },
                        Set.of(this))
                .onlyIf(() -> this.goal != IndexerGoal.DISABLED);
    }

    public Command disable() {
        return this.runOnce(() -> goal = IndexerGoal.DISABLED)
                .andThen(Commands.idle())
                .finallyDo(() -> goal = IndexerGoal.IDLE)
                .withName("Disable Indexer");
    }

    public IndexerGoal getGoal() {
        return goal;
    }

    public Command activate() {
        return Commands.sequence(
                        this.runOnce(() -> io.setFeedOutput(UNJAM_FEED_VOLTAGE)),
                        Commands.waitSeconds(0.1),
                        this.runOnce(() -> io.setFeedOutput(Volts.of(feedVoltage.get()))),
                        Commands.waitSeconds(0.3),
                        this.runOnce(() -> io.setSpinOutput(Volts.of(spinVoltage.get()))))
                // .andThen(Commands.waitUntil(() -> inputs.feedVelocity.abs(RPM) >= FEED_THRESHOLD.in(RPM)))
                // .andThen(this.runOnce(() -> io.setSpinOutput(Volts.of(spinVoltage.get()))))
                .withName("IndexerActivate");
    }

    private Command unjam() {
        return Commands.runOnce(() -> {
                    io.setFeedOutput(UNJAM_FEED_VOLTAGE);
                    io.setSpinOutput(UNJAM_SPIN_VOLTAGE);
                })
                .andThen(Commands.waitSeconds(0.3))
                .andThen(Commands.runOnce(() -> {
                    io.setFeedOutput(Volts.of(feedVoltage.get()));
                    io.setSpinOutput(Volts.of(spinVoltage.get()));
                }));
    }

    public void stop(boolean changeGoal) {
        io.stopSpin();
        io.stopFeed();
        if (changeGoal) this.goal = IndexerGoal.IDLE;
    }

    public void stop() {
        stop(true);
    }

    public enum IndexerGoal {
        ACTIVE,
        IDLE,
        DISABLED // manual override
    }
}
