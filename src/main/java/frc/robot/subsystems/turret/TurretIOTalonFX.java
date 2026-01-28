// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.Constants.CAN_FD_BUS;
import static frc.robot.Constants.TurretConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Add your docs here. */
public class TurretIOTalonFX implements TurretIO {
    private final TalonFX turnMotor;
    private final TalonFX hoodMotor;
    private final TalonFX flywheelMotor;
    private final TalonFX shootMotor;

    private final StatusSignal<Angle> turnPosition;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Voltage> turnAppliedVolts;
    private final StatusSignal<Current> turnCurrent;

    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Voltage> hoodAppliedVolts;
    private final StatusSignal<Current> hoodCurrent;

    private final StatusSignal<AngularVelocity> flywheelSpeed;
    private final StatusSignal<AngularAcceleration> flywheelAccel;
    private final StatusSignal<Double> flywheelSetpointSpeed;
    private final StatusSignal<Double> flywheelSetpointAccel;
    private final StatusSignal<Voltage> flywheelAppliedVolts;
    private final StatusSignal<Current> flywheelCurrent;

    private final PositionVoltage turnPositionRequest = new PositionVoltage(0);
    private final PositionVoltage hoodPositionRequest = new PositionVoltage(0);
    private final VelocityTorqueCurrentFOC flywheelVelocityRequest = new VelocityTorqueCurrentFOC(0);

    private final NeutralOut neutralOut = new NeutralOut();

    public TurretIOTalonFX() {
        turnMotor = new TalonFX(TURN_ID, CAN_FD_BUS);
        hoodMotor = new TalonFX(HOOD_ID, CAN_FD_BUS);
        flywheelMotor = new TalonFX(FLYWHEEL_ID, CAN_FD_BUS);
        shootMotor = new TalonFX(SHOOT_ID, CAN_FD_BUS);

        turnPosition = turnMotor.getPosition();
        turnVelocity = turnMotor.getVelocity();
        turnAppliedVolts = turnMotor.getMotorVoltage();
        turnCurrent = turnMotor.getStatorCurrent();

        hoodPosition = hoodMotor.getPosition();
        hoodVelocity = hoodMotor.getVelocity();
        hoodAppliedVolts = hoodMotor.getMotorVoltage();
        hoodCurrent = hoodMotor.getStatorCurrent();

        flywheelSpeed = flywheelMotor.getVelocity();
        flywheelAccel = flywheelMotor.getAcceleration();
        flywheelSetpointSpeed = flywheelMotor.getClosedLoopReference();
        flywheelSetpointAccel = flywheelMotor.getClosedLoopReferenceSlope();
        flywheelAppliedVolts = flywheelMotor.getMotorVoltage();
        flywheelCurrent = flywheelMotor.getTorqueCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                turnPosition,
                turnAppliedVolts,
                turnCurrent,
                hoodPosition,
                hoodAppliedVolts,
                hoodCurrent,
                flywheelSpeed,
                flywheelAppliedVolts,
                flywheelCurrent);
        turnMotor.optimizeBusUtilization();
        hoodMotor.optimizeBusUtilization();
        flywheelMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turnMotorConnected = BaseStatusSignal.refreshAll(turnPosition, turnAppliedVolts, turnCurrent)
                .isOK();
        inputs.turnPosition = turnPosition.getValue();
        inputs.turnVelocity = turnVelocity.getValue();
        inputs.turnAppliedVolts = turnAppliedVolts.getValue();
        inputs.turnCurrent = turnCurrent.getValue();

        inputs.hoodMotorConnected = BaseStatusSignal.refreshAll(hoodPosition, hoodAppliedVolts, hoodCurrent)
                .isOK();
        inputs.hoodPosition = hoodPosition.getValue();
        inputs.hoodVelocity = hoodVelocity.getValue();
        inputs.hoodAppliedVolts = hoodAppliedVolts.getValue();
        inputs.hoodCurrent = hoodCurrent.getValue();

        inputs.flywheelMotorConnected = BaseStatusSignal.refreshAll(
                        flywheelSpeed, flywheelAppliedVolts, flywheelCurrent)
                .isOK();
        inputs.flywheelSpeed = flywheelSpeed.getValue();
        inputs.flywheelAccel = flywheelAccel.getValue();
        inputs.flywheelSetpointSpeed = RotationsPerSecond.of(flywheelSetpointSpeed.getValueAsDouble());
        inputs.flywheelSetpointAccel = RotationsPerSecondPerSecond.of(flywheelSetpointAccel.getValueAsDouble());
        inputs.flywheelAppliedVolts = flywheelAppliedVolts.getValue();
        inputs.flywheelCurrent = flywheelCurrent.getValue();
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
    public void stopShoot() {
        shootMotor.setControl(neutralOut);
    }

    @Override
    public void resetTurnEncoder() {
        turnMotor.setPosition(turnPosition.getValue().in(Rotations) % 1);
    }
}
