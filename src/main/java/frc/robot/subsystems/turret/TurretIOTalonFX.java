// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.Constants.CAN_FD_BUS;
import static frc.robot.Constants.TurretConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class TurretIOTalonFX implements TurretIO {
    private final TalonFX turnMotor;
    private final TalonFX hoodMotor;
    private final TalonFX flywheelMotor;
    private final TalonFX flywheelFollowerMotor;

    private final CANcoder encoder;

    private final TalonFXConfiguration turnConfig;
    private final TalonFXConfiguration hoodConfig;
    private final TalonFXConfiguration flywheelConfig;
    private final TalonFXConfiguration flywheelFollowerConfig;

    private final StatusSignal<Angle> turnPosition;
    private final StatusSignal<Angle> turnEncoderPosition;
    private final StatusSignal<Double> turnSetpoint;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Voltage> turnAppliedVolts;
    private final StatusSignal<Current> turnCurrent;
    private final StatusSignal<Current> turnSupplyCurrent;

    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<Double> hoodSetpoint;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Voltage> hoodAppliedVolts;
    private final StatusSignal<Current> hoodCurrent;
    private final StatusSignal<Current> hoodSupplyCurrent;

    private final StatusSignal<AngularVelocity> flywheelSpeed;
    private final StatusSignal<AngularAcceleration> flywheelAccel;
    private final StatusSignal<Double> flywheelSetpointSpeed;
    private final StatusSignal<Double> flywheelSetpointAccel;
    private final StatusSignal<Voltage> flywheelAppliedVolts;
    private final StatusSignal<Current> flywheelCurrent;
    private final StatusSignal<Current> flywheelFollowerCurrent;
    private final StatusSignal<Current> flywheelSupplyCurrent;
    private final StatusSignal<Current> flywheelFollowerSupplyCurrent;

    private final PositionVoltage turnPositionRequest = new PositionVoltage(0).withUpdateFreqHz(250);
    private final PositionVoltage hoodPositionRequest = new PositionVoltage(0).withUpdateFreqHz(250);
    private final VoltageOut hoodVoltageRequest = new VoltageOut(0);
    private final VelocityTorqueCurrentFOC flywheelVelocityRequest =
            new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(250);

    private final Follower followRequest = new Follower(
                    FLYWHEEL_ID,
                    FLYWHEEL_OUTPUT_CONFIGS.Inverted == FLYWHEEL_FOLLOWER_OUTPUT_CONFIGS.Inverted
                            ? MotorAlignmentValue.Aligned
                            : MotorAlignmentValue.Opposed)
            .withUpdateFreqHz(250);
    private final NeutralOut neutralOut = new NeutralOut();

    public TurretIOTalonFX() {
        turnMotor = new TalonFX(TURN_ID, CAN_FD_BUS);
        hoodMotor = new TalonFX(HOOD_ID, CAN_FD_BUS);
        flywheelMotor = new TalonFX(FLYWHEEL_ID, CAN_FD_BUS);
        flywheelFollowerMotor = new TalonFX(FLYWHEEL_FOLLOWER_ID, CAN_FD_BUS);
        encoder = new CANcoder(ENCODER_ID, CAN_FD_BUS);

        turnConfig = new TalonFXConfiguration()
                .withFeedback(TURN_FEEDBACK_CONFIGS)
                .withSlot0(TURN_GAINS)
                .withCurrentLimits(TURN_CURRENT_LIMITS)
                .withMotorOutput(TURN_OUTPUT_CONFIGS)
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(MAX_TURN_ANGLE)
                        .withReverseSoftLimitEnable(true)
                        .withReverseSoftLimitThreshold(MIN_TURN_ANGLE));

        hoodConfig = new TalonFXConfiguration()
                .withSlot0(HOOD_GAINS)
                .withCurrentLimits(HOOD_CURRENT_LIMITS)
                .withMotorOutput(HOOD_OUTPUT_CONFIGS)
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(false)
                        .withForwardSoftLimitThreshold(MAX_HOOD_ANGLE)
                        .withReverseSoftLimitEnable(false)
                        .withReverseSoftLimitThreshold(MIN_HOOD_ANGLE))
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(HOOD_MOTOR_RATIO));

        flywheelConfig = new TalonFXConfiguration()
                .withSlot0(FLYWHEEL_GAINS)
                .withCurrentLimits(FLYWHEEL_CURRENT_LIMITS)
                .withMotorOutput(FLYWHEEL_OUTPUT_CONFIGS)
                .withFeedback(FLYWHEEL_FEEDBACK_CONFIGS);

        flywheelFollowerConfig = new TalonFXConfiguration()
                .withCurrentLimits(FLYWHEEL_CURRENT_LIMITS)
                .withMotorOutput(FLYWHEEL_FOLLOWER_OUTPUT_CONFIGS);

        PhoenixUtil.tryUntilOk(5, () -> turnMotor.getConfigurator().apply(turnConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> flywheelFollowerMotor.getConfigurator().apply(flywheelFollowerConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> encoder.getConfigurator().apply(ENCODER_CONFIGS));

        turnPosition = turnMotor.getPosition();
        turnEncoderPosition = encoder.getPosition();
        turnSetpoint = turnMotor.getClosedLoopReference();
        turnVelocity = turnMotor.getVelocity();
        turnAppliedVolts = turnMotor.getMotorVoltage();
        turnCurrent = turnMotor.getStatorCurrent();
        turnSupplyCurrent = turnMotor.getSupplyCurrent();

        hoodPosition = hoodMotor.getPosition();
        hoodSetpoint = hoodMotor.getClosedLoopReference();
        hoodVelocity = hoodMotor.getVelocity();
        hoodAppliedVolts = hoodMotor.getMotorVoltage();
        hoodCurrent = hoodMotor.getStatorCurrent();
        hoodSupplyCurrent = hoodMotor.getSupplyCurrent();

        flywheelSpeed = flywheelMotor.getVelocity();
        flywheelAccel = flywheelMotor.getAcceleration();
        flywheelSetpointSpeed = flywheelMotor.getClosedLoopReference();
        flywheelSetpointAccel = flywheelMotor.getClosedLoopReferenceSlope();
        flywheelAppliedVolts = flywheelMotor.getMotorVoltage();
        flywheelCurrent = flywheelMotor.getTorqueCurrent();
        flywheelSupplyCurrent = flywheelMotor.getSupplyCurrent();
        flywheelFollowerCurrent = flywheelFollowerMotor.getTorqueCurrent();
        flywheelFollowerSupplyCurrent = flywheelFollowerMotor.getSupplyCurrent();

        PhoenixUtil.registerStatusSignals(
                Hertz.of(50),
                // turnPosition,
                turnSetpoint,
                // turnVelocity,
                // turnAppliedVolts,
                turnCurrent,
                turnSupplyCurrent,
                hoodPosition,
                hoodSetpoint,
                hoodVelocity,
                hoodAppliedVolts,
                hoodCurrent,
                hoodSupplyCurrent,
                flywheelSpeed,
                flywheelAccel,
                flywheelSetpointSpeed,
                flywheelSetpointAccel,
                flywheelAppliedVolts,
                flywheelCurrent,
                flywheelSupplyCurrent,
                flywheelFollowerCurrent,
                flywheelFollowerSupplyCurrent);
        PhoenixUtil.registerStatusSignals(
                Hertz.of(250), turnPosition, turnEncoderPosition, turnVelocity, turnAppliedVolts);
        turnMotor.optimizeBusUtilization();
        hoodMotor.optimizeBusUtilization();
        flywheelMotor.optimizeBusUtilization();
        flywheelFollowerMotor.optimizeBusUtilization();

        flywheelFollowerMotor.setControl(followRequest);

        turnMotor.setPosition(Rotations.of(
                MathUtil.inputModulus(turnMotor.getRotorPosition().getValue().in(Rotations), -0.5, 0.5)
                        / TURN_TO_TURRET_RATIO));
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turnMotorConnected = BaseStatusSignal.isAllGood(
                turnPosition,
                turnEncoderPosition,
                turnSetpoint,
                turnVelocity,
                turnAppliedVolts,
                turnCurrent,
                turnSupplyCurrent);
        inputs.turnPosition = turnPosition.getValue();
        inputs.turnEncoderPosition = turnEncoderPosition.getValue().div(ENCODER_TO_TURRET_RATIO);
        inputs.turnSetpoint = Rotations.of(turnSetpoint.getValueAsDouble());
        inputs.turnVelocity = turnVelocity.getValue();
        inputs.turnAppliedVolts = turnAppliedVolts.getValue();
        inputs.turnCurrent = turnCurrent.getValue();
        inputs.turnSupplyCurrent = turnSupplyCurrent.getValue();

        inputs.hoodMotorConnected = BaseStatusSignal.isAllGood(
                hoodPosition, hoodSetpoint, hoodVelocity, hoodAppliedVolts, hoodCurrent, hoodSupplyCurrent);
        inputs.hoodPosition = hoodPosition.getValue();
        inputs.hoodSetpoint = Rotations.of(hoodSetpoint.getValueAsDouble());
        inputs.hoodVelocity = hoodVelocity.getValue();
        inputs.hoodAppliedVolts = hoodAppliedVolts.getValue();
        inputs.hoodCurrent = hoodCurrent.getValue();
        inputs.hoodSupplyCurrent = hoodSupplyCurrent.getValue();

        inputs.flywheelMotorConnected = BaseStatusSignal.isAllGood(
                flywheelSpeed,
                flywheelAccel,
                flywheelSetpointSpeed,
                flywheelSetpointAccel,
                flywheelAppliedVolts,
                flywheelCurrent,
                flywheelSupplyCurrent,
                flywheelFollowerCurrent,
                flywheelFollowerSupplyCurrent);
        inputs.flywheelSpeed = flywheelSpeed.getValue();
        inputs.flywheelAccel = flywheelAccel.getValue();
        inputs.flywheelSetpointSpeed = RotationsPerSecond.of(flywheelSetpointSpeed.getValueAsDouble());
        inputs.flywheelSetpointAccel = RotationsPerSecondPerSecond.of(flywheelSetpointAccel.getValueAsDouble());
        inputs.flywheelAppliedVolts = flywheelAppliedVolts.getValue();
        inputs.flywheelCurrent = flywheelCurrent.getValue().plus(flywheelFollowerCurrent.getValue());
        inputs.flywheelSupplyCurrent = flywheelSupplyCurrent.getValue().plus(flywheelFollowerSupplyCurrent.getValue());
        inputs.flywheelLeaderCurrent = flywheelCurrent.getValue();
        inputs.flywheelLeaderSupplyCurrent = flywheelSupplyCurrent.getValue();
        inputs.flywheelFollowerCurrent = flywheelFollowerCurrent.getValue();
        inputs.flywheelFollowerSupplyCurrent = flywheelFollowerSupplyCurrent.getValue();
    }

    @Override
    public void setTurnSetpoint(Angle position, AngularVelocity velocity) {
        turnMotor.setControl(turnPositionRequest.withPosition(position).withVelocity(velocity));
    }

    @Override
    public void setHoodAngle(Angle angle) {
        hoodMotor.setControl(hoodPositionRequest.withPosition(angle));
    }

    @Override
    public void setHoodOut(Voltage out) {
        hoodMotor.setControl(hoodVoltageRequest.withOutput(out));
    }

    @Override
    public void setFlywheelSpeed(AngularVelocity speed) {
        flywheelMotor.setControl(flywheelVelocityRequest.withVelocity(speed));
    }

    @Override
    public void stopTurn() {
        turnMotor.setControl(neutralOut);
    }

    @Override
    public void stopHood() {
        hoodMotor.setControl(neutralOut);
    }

    @Override
    public void stopFlywheel() {
        flywheelMotor.setControl(neutralOut);
    }

    @Override
    public void resetTurnEncoder() {
        turnMotor.setPosition(encoder.getPosition().getValue().div(ENCODER_TO_TURRET_RATIO));
    }

    @Override
    public void zeroHoodPosition() {
        hoodMotor.setPosition(MIN_HOOD_ANGLE);
    }

    @Override
    public void setTurnPID(double kP, double kD, double kV, double kS) {
        turnConfig.Slot0.kP = kP;
        turnConfig.Slot0.kD = kD;
        turnConfig.Slot0.kV = kV;
        turnConfig.Slot0.kS = kS;

        PhoenixUtil.tryUntilOk(5, () -> turnMotor.getConfigurator().apply(turnConfig.Slot0, 0.25));
    }

    @Override
    public void setHoodPID(double kP, double kD, double kS) {
        hoodConfig.Slot0.kP = kP;
        hoodConfig.Slot0.kD = kD;
        hoodConfig.Slot0.kS = kS;

        PhoenixUtil.tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig.Slot0, 0.25));
    }

    @Override
    public void setFlywheelPID(double kP, double kD, double kV, double kS) {
        flywheelConfig.Slot0.kP = kP;
        flywheelConfig.Slot0.kD = kD;
        flywheelConfig.Slot0.kV = kV;
        flywheelConfig.Slot0.kS = kS;

        PhoenixUtil.tryUntilOk(5, () -> flywheelMotor.getConfigurator().apply(flywheelConfig.Slot0, 0.25));
    }
}
