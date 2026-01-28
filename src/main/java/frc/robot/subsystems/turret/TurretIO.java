// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public boolean turnMotorConnected = false;
        public Voltage turnAppliedVolts = Volts.zero();
        public Current turnCurrent = Amps.zero();
        public Angle turnPosition = Radians.zero();
        public AngularVelocity turnVelocity = RadiansPerSecond.zero();

        public boolean hoodMotorConnected = false;
        public Voltage hoodAppliedVolts = Volts.zero();
        public Current hoodCurrent = Amps.zero();
        public Angle hoodPosition = Radians.zero();
        public AngularVelocity hoodVelocity = RadiansPerSecond.zero();

        public boolean flywheelMotorConnected = false;
        public Voltage flywheelAppliedVolts = Volts.zero();
        public Current flywheelCurrent = Amps.zero();
        public AngularVelocity flywheelSpeed = RadiansPerSecond.zero();
        public AngularAcceleration flywheelAccel = RadiansPerSecondPerSecond.zero();
        public AngularVelocity flywheelSetpointSpeed = RadiansPerSecond.zero();
        public AngularAcceleration flywheelSetpointAccel = RadiansPerSecondPerSecond.zero();
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void setTurnSetpoint(Angle position, AngularVelocity velocity) {}

    public default void setHoodAngle(Angle angle) {}

    public default void setFlywheelSpeed(AngularVelocity speed) {}

    public default void stopTurn() {}

    public default void stopHood() {}

    public default void stopFlywheel() {}

    public default void stopShoot() {}

    public default void resetTurnCRT() {}
}
