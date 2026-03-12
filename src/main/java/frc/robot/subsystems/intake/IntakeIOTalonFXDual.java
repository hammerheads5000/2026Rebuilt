// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static frc.robot.Constants.CAN_FD_BUS;
import static frc.robot.Constants.IntakeConstants.DEPLOY_POS;
import static frc.robot.Constants.IntakeConstants.PINION_PITCH_RADIUS;
import static frc.robot.Constants.IntakeConstants.RACK_GAINS;
import static frc.robot.Constants.IntakeConstants.RACK_MOTION_MAGIC;
import static frc.robot.Constants.IntakeConstants.RIGHT_RACK_CURRENT_LIMITS;
import static frc.robot.Constants.IntakeConstants.RIGHT_RACK_OUTPUT_CONFIGS;
import static frc.robot.Constants.IntakeConstants.ROTOR_TO_PINION_RATIO;
import static frc.robot.Constants.IntakeConstants.SPIN_CURRENT_LIMITS;
import static frc.robot.Constants.IntakeConstants.SPIN_OUTPUT_CONFIGS;
import static frc.robot.Constants.IntakeConstants.STOW_POS;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DifferentialMotionMagicVoltage;
import com.ctre.phoenix6.controls.DifferentialVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.DifferentialMotorConstants;
import com.ctre.phoenix6.mechanisms.SimpleDifferentialMechanism;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class IntakeIOTalonFXDual implements IntakeIO {
    private final TalonFX spinMotor;
    private final SimpleDifferentialMechanism<TalonFX> diffMech;

    private final TalonFXConfiguration rackConfig;
    private final TalonFXConfiguration spinConfig;
    private final DifferentialMotorConstants<TalonFXConfiguration> diffConfig;

    private final StatusSignal<Angle> rackPosition;
    private final StatusSignal<AngularVelocity> rackVelocity;
    private final StatusSignal<Double> rackSetpoint;
    private final StatusSignal<Double> rackSetpointVelocity;
    private final StatusSignal<Current> rackCurrent;
    private final StatusSignal<Current> rackSupplyCurrent;
    private final StatusSignal<Voltage> rackAppliedVolts;

    private final StatusSignal<AngularVelocity> spinVelocity;
    private final StatusSignal<Current> spinCurrent;
    private final StatusSignal<Current> spinSupplyCurrent;
    private final StatusSignal<Voltage> spinAppliedVolts;

    private final DifferentialMotionMagicVoltage rackPositionRequest = new DifferentialMotionMagicVoltage(0, 0);
    private final PositionVoltage rackDiffRequest = new PositionVoltage(0);
    private final DifferentialVoltage rackVoltageRequest = new DifferentialVoltage(0, 0);
    private final VoltageOut spinVoltageRequest = new VoltageOut(0).withEnableFOC(false);

    private final NeutralOut neutralOut = new NeutralOut();

    public IntakeIOTalonFXDual(int rackFrontID, int rackBackID, int spinID) {
        this.spinMotor = new TalonFX(spinID, Constants.CAN_FD_BUS);

        rackConfig = new TalonFXConfiguration()
                .withSlot0(RACK_GAINS)
                .withMotorOutput(RIGHT_RACK_OUTPUT_CONFIGS)
                .withCurrentLimits(RIGHT_RACK_CURRENT_LIMITS)
                .withMotionMagic(RACK_MOTION_MAGIC)
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withForwardSoftLimitEnable(false)
                        .withForwardSoftLimitThreshold(distanceToRotorAngle(DEPLOY_POS))
                        .withReverseSoftLimitEnable(false)
                        .withReverseSoftLimitThreshold(distanceToRotorAngle(STOW_POS)))
                .withFeedback(new FeedbackConfigs()
                        .withRotorToSensorRatio(1)
                        .withSensorToMechanismRatio(ROTOR_TO_PINION_RATIO));

        diffConfig = new DifferentialMotorConstants<TalonFXConfiguration>()
                .withCANBusName(CAN_FD_BUS.getName())
                .withLeaderId(rackBackID)
                .withFollowerId(rackFrontID)
                .withSensorToDifferentialRatio(1)
                .withAlignment(MotorAlignmentValue.Opposed)
                .withLeaderInitialConfigs(rackConfig)
                .withFollowerInitialConfigs(new TalonFXConfiguration().withFeedback(rackConfig.Feedback))
                .withFollowerUsesCommonLeaderConfigs(true);

        spinConfig = new TalonFXConfiguration()
                .withCurrentLimits(SPIN_CURRENT_LIMITS)
                .withMotorOutput(SPIN_OUTPUT_CONFIGS);

        PhoenixUtil.tryUntilOk(5, () -> spinMotor.getConfigurator().apply(spinConfig, 0.25));
        diffMech = new SimpleDifferentialMechanism<TalonFX>(TalonFX::new, diffConfig);

        diffMech.setPosition(0, 0);

        this.rackPosition = diffMech.getAveragePosition();
        this.rackVelocity = diffMech.getAverageVelocity();
        this.rackSetpoint = diffMech.getAverageClosedLoopReference();
        this.rackSetpointVelocity = diffMech.getAverageClosedLoopReferenceSlope();
        this.rackCurrent = diffMech.getLeader().getTorqueCurrent();
        this.rackSupplyCurrent = diffMech.getLeader().getSupplyCurrent();
        this.rackAppliedVolts = diffMech.getLeader().getMotorVoltage();

        this.spinVelocity = spinMotor.getVelocity();
        this.spinCurrent = spinMotor.getStatorCurrent();
        this.spinSupplyCurrent = spinMotor.getSupplyCurrent();
        this.spinAppliedVolts = spinMotor.getMotorVoltage();

        PhoenixUtil.registerStatusSignals(
                Hertz.of(50),
                rackPosition,
                rackVelocity,
                rackSetpoint,
                rackSetpointVelocity,
                rackCurrent,
                rackSupplyCurrent,
                rackAppliedVolts,
                spinVelocity,
                spinCurrent,
                spinSupplyCurrent,
                spinAppliedVolts);
        // diffMech.getLeader().optimizeBusUtilization();
        // diffMech.getFollower().optimizeBusUtilization();
        spinMotor.optimizeBusUtilization();
    }

    public static Distance rotorAngleToDistance(Angle rotorAngle) {
        return PINION_PITCH_RADIUS.times(rotorAngle.in(Radians));
    }

    public static Angle distanceToRotorAngle(Distance distance) {
        return Radians.of(distance.in(Meters) / PINION_PITCH_RADIUS.in(Meters));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rackMotorConnected = BaseStatusSignal.isAllGood(
                rackPosition,
                rackVelocity,
                rackSetpoint,
                rackSetpointVelocity,
                rackCurrent,
                rackSupplyCurrent,
                rackAppliedVolts);
        inputs.rackPosition = rotorAngleToDistance(rackPosition.getValue());
        inputs.rackVelocity = rotorAngleToDistance(
                        Radians.of(rackVelocity.getValue().in(RadiansPerSecond)))
                .per(Second);
        inputs.rackSetpoint = rotorAngleToDistance(Rotations.of(rackSetpoint.getValueAsDouble()));
        inputs.rackSetpointVelocity = rotorAngleToDistance(Rotations.of(rackSetpointVelocity.getValue()))
                .per(Second);
        inputs.rackCurrent = rackCurrent.getValue();
        inputs.rackSupplyCurrent = rackSupplyCurrent.getValue();
        inputs.rackAppliedVolts = rackAppliedVolts.getValue();

        inputs.spinMotorConnected =
                BaseStatusSignal.isAllGood(spinVelocity, spinCurrent, spinSupplyCurrent, spinAppliedVolts);
        inputs.spinVelocity = spinVelocity.getValue();
        inputs.spinCurrent = spinCurrent.getValue();
        inputs.spinSupplyCurrent = spinSupplyCurrent.getValue();
        inputs.spinAppliedVolts = spinAppliedVolts.getValue();
    }

    @Override
    public void setRackPosition(Distance position) {
        diffMech.setControl(rackPositionRequest.withAveragePosition(distanceToRotorAngle(position)));
    }

    @Override
    public void setRackOutput(Voltage out) {
        diffMech.setControl(rackVoltageRequest.withAverageOutput(out));
    }

    @Override
    public void setSpinOutput(Voltage volts) {
        spinMotor.setControl(spinVoltageRequest.withOutput(volts));
    }

    @Override
    public void stopRack() {
        diffMech.setCoastOut();
    }

    @Override
    public void stopSpin() {
        spinMotor.setControl(neutralOut);
    }

    @Override
    public void zeroPosition() {
        diffMech.setPosition(0, 0);
    }

    @Override
    public void setRackPID(double kP, double kD, double kV, double kA, double kS, double maxVel, double maxAcc) {
        rackConfig.Slot0.kP = kP;
        rackConfig.Slot0.kD = kD;
        rackConfig.Slot0.kV = kV;
        rackConfig.Slot0.kA = kA;
        rackConfig.Slot0.kS = kS;
        rackConfig.MotionMagic.MotionMagicCruiseVelocity = maxVel;
        rackConfig.MotionMagic.MotionMagicAcceleration = maxAcc;

        tryUntilOk(5, () -> diffMech.getLeader().getConfigurator().apply(rackConfig));
    }
}
