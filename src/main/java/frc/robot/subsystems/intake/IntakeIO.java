// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean rackMotorConnected = false;
        public Distance rackPosition = Meters.of(0.0);
        public LinearVelocity rackVelocity = MetersPerSecond.of(0.0);
        public Distance rackSetpoint = Meters.of(0.0);
        public LinearVelocity rackSetpointVelocity = MetersPerSecond.of(0.0);
        public Current rackCurrent = Amps.of(0.0);
        public Current rackSupplyCurrent = Amps.of(0.0);
        public Voltage rackAppliedVolts = Volts.of(0.0);

        public boolean spinMotorConnected = false;
        public AngularVelocity spinVelocity = RadiansPerSecond.of(0.0);
        public Current spinCurrent = Amps.of(0.0);
        public Current spinSupplyCurrent = Amps.of(0.0);
        public Voltage spinAppliedVolts = Volts.of(0.0);
        public Temperature spinTemp = Celsius.of(0);
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setRackPosition(Distance position) {}

    public default void setRackOutput(Voltage out) {}

    public default void setSpinOutput(Voltage out) {}

    public default void stopRack() {}

    public default void stopSpin() {}

    public default void zeroPosition() {}

    public default void setRackPID(
            double kP, double kD, double kV, double kA, double kS, double maxVel, double maxAcc) {}
}
