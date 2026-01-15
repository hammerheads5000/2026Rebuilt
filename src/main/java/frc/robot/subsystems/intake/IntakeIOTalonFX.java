// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/** Add your docs here. */
public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX rackMotor;
    private final TalonFX spinMotor;

    private final StatusSignal<Angle> rackPosition;
    private final StatusSignal<AngularVelocity> rackVelocity;
    private final StatusSignal<Current> rackCurrent;

    private final StatusSignal<AngularVelocity> spinVelocity;
    private final StatusSignal<Current> spinCurrent;

    private final VoltageOut rackVoltageRequest = new VoltageOut(0);
    private final VoltageOut spinVoltageRequest = new VoltageOut(0);

    private final NeutralOut neutralOut = new NeutralOut();

    public IntakeIOTalonFX(int rackID, int spinID) {
        this.rackMotor = new TalonFX(rackID, Constants.CAN_FD_BUS);
        this.spinMotor = new TalonFX(spinID, Constants.CAN_FD_BUS);

        this.rackPosition = rackMotor.getPosition();
        this.rackVelocity = rackMotor.getVelocity();
        this.rackCurrent = rackMotor.getStatorCurrent();
        this.spinVelocity = spinMotor.getVelocity();
        this.spinCurrent = spinMotor.getStatorCurrent();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rackMotorConnected = BaseStatusSignal.refreshAll(rackPosition).isOK();
        inputs.rackPosition = Meters.of(rackPosition.getValueAsDouble());
        inputs.rackVelocity = MetersPerSecond.of(rackVelocity.getValueAsDouble());
        inputs.rackCurrent = rackCurrent.getValue();
        inputs.spinMotorConnected = BaseStatusSignal.refreshAll(spinVelocity).isOK();
        inputs.spinVelocity = spinVelocity.getValue();
        inputs.spinCurrent = spinCurrent.getValue();
    }

    @Override
    public void setRackOutput(Voltage volts) {
        rackMotor.setControl(rackVoltageRequest.withOutput(volts));
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
}
