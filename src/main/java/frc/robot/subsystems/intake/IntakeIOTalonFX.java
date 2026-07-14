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
import static frc.robot.Constants.IntakeConstants.LEFT_RACK_OUTPUT_CONFIGS;
import static frc.robot.Constants.IntakeConstants.LEFT_ROTOR_TO_PINION_RATIO;
import static frc.robot.Constants.IntakeConstants.PINION_PITCH_RADIUS;
import static frc.robot.Constants.IntakeConstants.RACK_CURRENT_LIMITS;
import static frc.robot.Constants.IntakeConstants.RACK_GAINS;
import static frc.robot.Constants.IntakeConstants.RACK_MOTION_MAGIC;
import static frc.robot.Constants.IntakeConstants.RIGHT_RACK_ID;
import static frc.robot.Constants.IntakeConstants.RIGHT_RACK_OUTPUT_CONFIGS;
import static frc.robot.Constants.IntakeConstants.RIGHT_ROTOR_TO_PINION_RATIO;
import static frc.robot.Constants.IntakeConstants.SPIN_CURRENT_LIMITS;
import static frc.robot.Constants.IntakeConstants.SPIN_OUTPUT_CONFIGS;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

/** Add your docs here. */
public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX rackMotor;
    private final TalonFX spinMotor;

    private final TalonFXConfiguration rackConfig;
    private final TalonFXConfiguration spinConfig;

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
    private final StatusSignal<Temperature> spinTemp;

    private final MotionMagicVoltage rackPositionRequest = new MotionMagicVoltage(0);
    private final VoltageOut rackVoltageRequest = new VoltageOut(0);
    private final VoltageOut spinVoltageRequest = new VoltageOut(0).withEnableFOC(false);

    private final NeutralOut neutralOut = new NeutralOut();

    public IntakeIOTalonFX(int rackID, int spinID) {
        this.rackMotor = new TalonFX(rackID, Constants.CAN_FD_BUS);
        this.spinMotor = new TalonFX(spinID, Constants.CAN_FD_BUS);

        double ratio = rackID == RIGHT_RACK_ID ? RIGHT_ROTOR_TO_PINION_RATIO : LEFT_ROTOR_TO_PINION_RATIO;
        Slot0Configs gains = RACK_GAINS;

        if (rackID == RIGHT_RACK_ID) {
            gains.kP *= RIGHT_ROTOR_TO_PINION_RATIO / LEFT_ROTOR_TO_PINION_RATIO;
            gains.kD *= RIGHT_ROTOR_TO_PINION_RATIO / LEFT_ROTOR_TO_PINION_RATIO;
            gains.kV *= RIGHT_ROTOR_TO_PINION_RATIO / LEFT_ROTOR_TO_PINION_RATIO;
            gains.kA *= RIGHT_ROTOR_TO_PINION_RATIO / LEFT_ROTOR_TO_PINION_RATIO;
            gains.kS *= RIGHT_ROTOR_TO_PINION_RATIO / LEFT_ROTOR_TO_PINION_RATIO;
        }

        rackConfig = new TalonFXConfiguration()
                .withSlot0(gains)
                .withMotorOutput(rackID == RIGHT_RACK_ID ? RIGHT_RACK_OUTPUT_CONFIGS : LEFT_RACK_OUTPUT_CONFIGS)
                .withCurrentLimits(RACK_CURRENT_LIMITS)
                .withMotionMagic(RACK_MOTION_MAGIC)
                .withFeedback(new FeedbackConfigs().withRotorToSensorRatio(1).withSensorToMechanismRatio(ratio));

        spinConfig = new TalonFXConfiguration()
                .withCurrentLimits(SPIN_CURRENT_LIMITS)
                .withMotorOutput(SPIN_OUTPUT_CONFIGS);

        PhoenixUtil.tryUntilOk(5, () -> rackMotor.getConfigurator().apply(rackConfig, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> spinMotor.getConfigurator().apply(spinConfig, 0.25));

        rackMotor.setPosition(0);

        this.rackPosition = rackMotor.getPosition();
        this.rackVelocity = rackMotor.getVelocity();
        this.rackSetpoint = rackMotor.getClosedLoopReference();
        this.rackSetpointVelocity = rackMotor.getClosedLoopReferenceSlope();
        this.rackCurrent = rackMotor.getTorqueCurrent();
        this.rackSupplyCurrent = rackMotor.getSupplyCurrent();
        this.rackAppliedVolts = rackMotor.getMotorVoltage();

        this.spinVelocity = spinMotor.getVelocity();
        this.spinCurrent = spinMotor.getStatorCurrent();
        this.spinSupplyCurrent = spinMotor.getSupplyCurrent();
        this.spinAppliedVolts = spinMotor.getMotorVoltage();
        this.spinTemp = spinMotor.getDeviceTemp();

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
        rackMotor.optimizeBusUtilization();
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
                BaseStatusSignal.isAllGood(spinVelocity, spinCurrent, spinSupplyCurrent, spinAppliedVolts, spinTemp);
        inputs.spinVelocity = spinVelocity.getValue();
        inputs.spinCurrent = spinCurrent.getValue();
        inputs.spinSupplyCurrent = spinSupplyCurrent.getValue();
        inputs.spinAppliedVolts = spinAppliedVolts.getValue();
        inputs.spinTemp = spinTemp.getValue();
    }

    @Override
    public void setRackPosition(Distance position) {
        rackMotor.setControl(rackPositionRequest.withPosition(distanceToRotorAngle(position)));
    }

    @Override
    public void setRackOutput(Voltage out) {
        rackMotor.setControl(rackVoltageRequest.withOutput(out));
    }

    @Override
    public void setSpinOutput(Voltage volts) {
        spinMotor.setControl(spinVoltageRequest.withOutput(volts));
    }

    @Override
    public void stopRack() {
        rackMotor.setControl(neutralOut);
    }

    @Override
    public void stopSpin() {
        spinMotor.setControl(neutralOut);
    }

    @Override
    public void zeroPosition() {
        rackMotor.setPosition(0);
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

        tryUntilOk(5, () -> rackMotor.getConfigurator().apply(rackConfig));
    }
}
