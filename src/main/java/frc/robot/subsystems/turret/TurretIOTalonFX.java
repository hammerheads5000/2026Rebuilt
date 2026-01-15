// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.Constants.CAN_FD_BUS;
import static frc.robot.Constants.TurretConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
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
    private final StatusSignal<Current> turnCurrent;

    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Current> hoodCurrent;

    private final StatusSignal<AngularVelocity> flywheelSpeed;
    private final StatusSignal<Current> flywheelCurrent;

    private final StatusSignal<AngularVelocity> shootSpeed;
    private final StatusSignal<Current> shootCurrent;

    private final VoltageOut turnVoltageRequest = new VoltageOut(0);
    private final VoltageOut hoodVoltageRequest = new VoltageOut(0);
    private final VoltageOut flywheelVoltageRequest = new VoltageOut(0);
    private final VoltageOut shootVoltageRequest = new VoltageOut(0);

    private final NeutralOut neutralOut = new NeutralOut();

    public TurretIOTalonFX() {
        turnMotor = new TalonFX(TURN_ID, CAN_FD_BUS);
        hoodMotor = new TalonFX(HOOD_ID, CAN_FD_BUS);
        flywheelMotor = new TalonFX(FLYWHEEL_ID, CAN_FD_BUS);
        shootMotor = new TalonFX(SHOOT_ID, CAN_FD_BUS);

        turnPosition = turnMotor.getPosition();
        turnVelocity = turnMotor.getVelocity();
        turnCurrent = turnMotor.getStatorCurrent();

        hoodPosition = hoodMotor.getPosition();
        hoodVelocity = hoodMotor.getVelocity();
        hoodCurrent = hoodMotor.getStatorCurrent();

        flywheelSpeed = flywheelMotor.getVelocity();
        flywheelCurrent = flywheelMotor.getStatorCurrent();

        shootSpeed = shootMotor.getVelocity();
        shootCurrent = shootMotor.getStatorCurrent();
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turnMotorConnected = BaseStatusSignal.refreshAll(turnPosition, turnVelocity, turnCurrent)
                .isOK();
        inputs.turnPosition = turnPosition.getValue();
        inputs.turnVelocity = turnVelocity.getValue();
        inputs.turnCurrent = turnCurrent.getValue();

        inputs.hoodMotorConnected = BaseStatusSignal.refreshAll(hoodPosition, hoodVelocity, hoodCurrent)
                .isOK();
        inputs.hoodPosition = hoodPosition.getValue();
        inputs.hoodVelocity = hoodVelocity.getValue();
        inputs.hoodCurrent = hoodCurrent.getValue();

        inputs.flywheelMotorConnected =
                BaseStatusSignal.refreshAll(flywheelSpeed, flywheelCurrent).isOK();
        inputs.flywheelSpeed = flywheelSpeed.getValue();
        inputs.flywheelCurrent = flywheelCurrent.getValue();

        inputs.shootMotorConnected =
                BaseStatusSignal.refreshAll(shootSpeed, shootCurrent).isOK();
        inputs.shootSpeed = shootSpeed.getValue();
        inputs.shootCurrent = shootCurrent.getValue();
    }

    @Override
    public void setTurnOutput(Voltage out) {
        turnMotor.setControl(turnVoltageRequest.withOutput(out));
    }

    @Override
    public void setHoodOutput(Voltage out) {
        hoodMotor.setControl(hoodVoltageRequest.withOutput(out));
    }

    @Override
    public void setFlywheelOutput(Voltage out) {
        flywheelMotor.setControl(flywheelVoltageRequest.withOutput(out));
    }

    @Override
    public void setShootOutput(Voltage out) {
        shootMotor.setControl(shootVoltageRequest.withOutput(out));
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
