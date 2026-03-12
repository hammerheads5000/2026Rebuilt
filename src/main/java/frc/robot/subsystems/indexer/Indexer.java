package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IndexerConstants.FEED_VOLTAGE;
import static frc.robot.Constants.IndexerConstants.SPIN_VOLTAGE;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;
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

    private final IndexerVisualizer visualizer;

    private final Alert feedDisconnectedAlert = new Alert("Indexer Feed Motor Disconnected", AlertType.kError);
    private final Alert hookDisconnectedAlert = new Alert("Indexer Hook Motor Disconnected", AlertType.kError);

    @AutoLogOutput
    private IndexerGoal goal = IndexerGoal.IDLE;

    public Indexer(IndexerIO io) {
        this.io = io;
        this.visualizer = new IndexerVisualizer();

        SmartDashboard.putData("Overrides/Indexer", disable());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);

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
                                toSchedule = activate();
                            } else if (goal == IndexerGoal.IDLE) {
                                toSchedule = this.run(this::stop);
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
        return this.runOnce(() -> {
                    io.setFeedOutput(FEED_VOLTAGE);
                    io.setSpinOutput(SPIN_VOLTAGE);
                })
                // .andThen(Commands.waitSeconds(0.1875))
                // .andThen(this.runOnce(() -> {
                //     io.setFeedOutput(Volts.of(feedVoltage.get()));
                //     io.stopSpin();
                // }))
                // .andThen(Commands.waitUntil(() -> inputs.feedVelocity.abs(RPM) >= FEED_THRESHOLD.in(RPM)))
                // .andThen(this.runOnce(() -> io.setSpinOutput(Volts.of(spinVoltage.get()))))
                .withName("IndexerActivate");
    }

    public void stop() {
        io.stopSpin();
        io.stopFeed();
        this.goal = IndexerGoal.IDLE;
    }

    public enum IndexerGoal {
        ACTIVE,
        IDLE,
        DISABLED // manual override
    }
}
