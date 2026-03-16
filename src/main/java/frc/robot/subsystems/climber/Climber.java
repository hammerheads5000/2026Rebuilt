package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ClimberConstants.AUTO_CLIMB_POSITION;
import static frc.robot.Constants.ClimberConstants.CLIMB_POSITION;
import static frc.robot.Constants.ClimberConstants.CLIMB_VOLTAGE;
import static frc.robot.Constants.ClimberConstants.EXTEND_POSITION_BACK;
import static frc.robot.Constants.ClimberConstants.EXTEND_POSITION_FRONT;
import static frc.robot.Constants.ClimberConstants.EXTEND_VOLTAGE;
import static frc.robot.Constants.ClimberConstants.STALL_ANGULAR_VELOCITY;
import static frc.robot.Constants.ClimberConstants.STALL_CURRENT;
import static frc.robot.Constants.ClimberConstants.STOW_POSITION;
import static frc.robot.Constants.ClimberConstants.STOW_SLOW_POSITION;
import static frc.robot.Constants.ClimberConstants.STOW_SLOW_VOLTAGE;
import static frc.robot.Constants.ClimberConstants.STOW_VOLTAGE;
import static frc.robot.Constants.ClimberConstants.ZERO_VOLTAGE;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.VirtualPD;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
    private final ClimberIO io;

    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final ClimberVisualizer visualizer = new ClimberVisualizer();

    private boolean disabled = false;

    private final Alert frontDisconnectedAlert = new Alert("Climber Front Motor Disconnected", AlertType.kError);
    private final Alert backDisconnectedAlert = new Alert("Climber Front Motor Disconnected", AlertType.kError);

    public Climber(ClimberIO io) {
        this.io = io;

        VirtualPD.registerMotor(() -> inputs.frontSupplyCurrent, "Climb");
        VirtualPD.registerMotor(() -> inputs.backSupplyCurrent, "Climb");

        SmartDashboard.putData("Climb/Climb", climb());
        SmartDashboard.putData("Climb/AutoClimb", autoClimb());
        SmartDashboard.putData("Climb/Stow", stow());
        SmartDashboard.putData("Climb/Extend", extend());
        SmartDashboard.putData("Climb/Zero", zero());

        SmartDashboard.putData("Overrides/Climber", disable());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        Logger.recordOutput(
                "Climber/Current Command",
                this.getCurrentCommand() == null
                        ? "None"
                        : this.getCurrentCommand().getName());
        visualizer.update(inputs.frontPosition, inputs.backPosition);

        frontDisconnectedAlert.set(!inputs.frontConnected && Constants.CURRENT_MODE != Mode.SIM);
        backDisconnectedAlert.set(!inputs.backConnected && Constants.CURRENT_MODE != Mode.SIM);
    }

    private Command setVoltage(Voltage out) {
        return this.runOnce(() -> io.setVoltage(out));
    }

    private Command stop() {
        return this.runOnce(io::stop);
    }

    public boolean isExtended() {
        return inputs.frontPosition.gte(EXTEND_POSITION_FRONT) && inputs.backPosition.gte(EXTEND_POSITION_BACK);
    }

    public Command climb() {
        return Commands.sequence(
                        setVoltage(CLIMB_VOLTAGE),
                        Commands.waitUntil(() -> inputs.averagePosition.lte(CLIMB_POSITION)),
                        setVoltage(Volts.of(-0.5)),
                        Commands.idle())
                .finallyDo(io::stop)
                .unless(() -> disabled);
    }

    public Command autoClimb() {
        return Commands.sequence(
                        setVoltage(CLIMB_VOLTAGE),
                        Commands.waitUntil(() -> inputs.averagePosition.lte(AUTO_CLIMB_POSITION)),
                        setVoltage(Volts.of(-0.5)),
                        Commands.idle())
                .finallyDo(io::stop)
                .unless(() -> disabled);
    }

    public Command stow() {
        return Commands.sequence(
                        setVoltage(STOW_VOLTAGE),
                        Commands.waitUntil(() -> inputs.frontPosition.lte(STOW_SLOW_POSITION)
                                || inputs.backPosition.lte(STOW_SLOW_POSITION)),
                        setVoltage(STOW_SLOW_VOLTAGE),
                        Commands.parallel(
                                Commands.waitUntil(() -> inputs.frontPosition.lte(STOW_POSITION))
                                        .finallyDo(io::stopFront),
                                Commands.waitUntil(() -> inputs.backPosition.lte(STOW_POSITION))
                                        .finallyDo(io::stopBack)),
                        stop())
                .finallyDo(io::stop)
                .unless(() -> disabled);
    }

    public Command extend() {
        return Commands.sequence(
                        setVoltage(EXTEND_VOLTAGE),
                        Commands.parallel(
                                Commands.waitUntil(() -> inputs.frontPosition.gte(EXTEND_POSITION_FRONT))
                                        .finallyDo(io::stopFront),
                                Commands.waitUntil(() -> inputs.backPosition.gte(EXTEND_POSITION_BACK))
                                        .finallyDo(io::stopBack)),
                        stop())
                .finallyDo(io::stop)
                .unless(() -> disabled);
    }

    public Command zero() {
        return Commands.sequence(
                        this.runOnce(() -> {
                            io.setFrontVoltage(ZERO_VOLTAGE);
                            io.setBackVoltage(ZERO_VOLTAGE);
                        }),
                        Commands.waitSeconds(0.1),
                        Commands.parallel(
                                Commands.waitUntil(() -> inputs.frontTorqueCurrent.abs(Amps) > STALL_CURRENT.abs(Amps)
                                                && inputs.frontVelocity.abs(RadiansPerSecond)
                                                        < STALL_ANGULAR_VELOCITY.abs(RadiansPerSecond))
                                        .finallyDo(() -> io.stopFront()),
                                Commands.waitUntil(() -> inputs.backTorqueCurrent.abs(Amps) > STALL_CURRENT.abs(Amps)
                                                && inputs.backVelocity.abs(RadiansPerSecond)
                                                        < STALL_ANGULAR_VELOCITY.abs(RadiansPerSecond))
                                        .finallyDo(() -> io.stopBack())),
                        Commands.waitSeconds(0.4),
                        this.runOnce(io::zeroPosition))
                .finallyDo(io::stop)
                .unless(() -> disabled);
    }

    public Command disable() {
        return this.runOnce(() -> disabled = true)
                .andThen(Commands.idle())
                .finallyDo(() -> disabled = false)
                .withName("Disable Climber");
    }
}
