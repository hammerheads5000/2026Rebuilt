// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

/** Add your docs here. */
public class TurretIOSim implements TurretIO {
    private final DCMotor turnMotor = DCMotor.getKrakenX44Foc(1);
    private final LinearSystemSim<N2, N1, N2> turnSim =
            new LinearSystemSim<N2, N1, N2>(LinearSystemId.createDCMotorSystem(turnMotor, 0.005, 5), 0.02);

    private final DCMotor hoodMotor = DCMotor.getKrakenX44Foc(1);
    private final LinearSystemSim<N2, N1, N2> hoodSim =
            new LinearSystemSim<N2, N1, N2>(LinearSystemId.createDCMotorSystem(hoodMotor, 0.005, 2), 0.02);

    private final DCMotor flywheelMotor = DCMotor.getKrakenX60Foc(1);
    private FlywheelSim flywheelSim =
            new FlywheelSim(LinearSystemId.createFlywheelSystem(flywheelMotor, 0.005, 2), flywheelMotor, 0.01);

    private final DCMotor shootMotor = DCMotor.getKrakenX60Foc(1);
    private FlywheelSim shootSim =
            new FlywheelSim(LinearSystemId.createFlywheelSystem(shootMotor, 0.001, 2), shootMotor, 0.01);

    public TurretIOSim() {}

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        turnSim.update(0.02);
        hoodSim.update(0.02);
        flywheelSim.update(0.02);
        shootSim.update(0.02);

        // Simulate input values
        inputs.turnPosition = Radians.of(turnSim.getOutput(1));
        inputs.turnVelocity = RadiansPerSecond.of(turnSim.getOutput(0));
        inputs.turnCurrent = Amps.of(turnMotor.getCurrent(turnSim.getOutput(0), turnSim.getInput(0)));
        inputs.hoodPosition = Radians.of(hoodSim.getOutput(1));
        inputs.hoodVelocity = RadiansPerSecond.of(hoodSim.getOutput(0));
        inputs.hoodCurrent = Amps.of(hoodMotor.getCurrent(hoodSim.getOutput(0), hoodSim.getInput(0)));
        inputs.flywheelSpeed = flywheelSim.getAngularVelocity();
        inputs.flywheelCurrent = Amps.of(flywheelSim.getCurrentDrawAmps());
        inputs.shootSpeed = shootSim.getAngularVelocity();
        inputs.shootCurrent = Amps.of(shootSim.getCurrentDrawAmps());
    }

    @Override
    public void setTurnOutput(Voltage out) {
        turnSim.setInput(out.in(Volts));
    }

    @Override
    public void setHoodOutput(Voltage out) {
        hoodSim.setInput(out.in(Volts));
    }

    @Override
    public void setFlywheelOutput(Voltage out) {
        flywheelSim.setInputVoltage(out.in(Volts));
    }

    @Override
    public void setShootOutput(Voltage out) {
        shootSim.setInputVoltage(out.in(Volts));
    }
}
