// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public boolean rackMotorConnected = false;
        public Distance rackPosition = Meters.zero();
        public LinearVelocity rackVelocity = MetersPerSecond.zero();
        public Distance rackSetpoint = Meters.zero();
        public LinearVelocity rackSetpointVelocity = MetersPerSecond.zero();
        public Current rackCurrent = Amps.zero();
        public Voltage rackAppliedVolts = Volts.zero();

        public boolean spinMotorConnected = false;
        public AngularVelocity spinVelocity = RadiansPerSecond.zero();
        public Current spinCurrent = Amps.zero();
        public Voltage spinAppliedVolts = Volts.zero();
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setRackPosition(Distance position) {}

    public default void setSpinOutput(Voltage out) {}

    public default void stopRack() {}

    public default void stopSpin() {}
}
