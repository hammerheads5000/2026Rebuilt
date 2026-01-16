// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class IntakeIOSim implements IntakeIO {
    private final DCMotor rackMotor = DCMotor.getKrakenX44Foc(1);
    private final ElevatorSim rackSim = new ElevatorSim(
            LinearSystemId.createElevatorSystem(rackMotor, 0.2, 0.02, 2), rackMotor, 0, 0.4, false, 0, 0.03);

    private final DCMotor spinMotor = DCMotor.getKrakenX44Foc(1);
    private final DCMotorSim spinMotorSim =
            new DCMotorSim(LinearSystemId.createDCMotorSystem(spinMotor, 0.0005, 3), spinMotor, 0.1);

    /** Creates a new IntakeIOSim. */
    public IntakeIOSim() {}

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        rackSim.update(0.02);
        spinMotorSim.update(0.02);

        inputs.rackMotorConnected = true;
        inputs.rackPosition = Meters.of(rackSim.getPositionMeters());
        inputs.rackVelocity = MetersPerSecond.of(rackSim.getVelocityMetersPerSecond());
        inputs.rackCurrent = Amps.of(rackSim.getCurrentDrawAmps());

        inputs.spinMotorConnected = true;
        inputs.spinVelocity = spinMotorSim.getAngularVelocity();
        inputs.spinCurrent = Amps.of(spinMotorSim.getCurrentDrawAmps());
    }

    @Override
    public void setRackOutput(Voltage out) {
        rackSim.setInputVoltage(out.in(Volts));
    }

    @Override
    public void setSpinOutput(Voltage out) {
        spinMotorSim.setInputVoltage(out.in(Volts));
    }

    @Override
    public void stopRack() {
        rackSim.setInputVoltage(0);
    }

    @Override
    public void stopSpin() {
        spinMotorSim.setInputVoltage(0);
    }
}
