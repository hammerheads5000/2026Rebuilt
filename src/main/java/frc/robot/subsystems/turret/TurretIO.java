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
        public Voltage turnAppliedVolts = Volts.of(0.0);
        public Current turnCurrent = Amps.of(0.0);
        public Current turnSupplyCurrent = Amps.of(0.0);
        public Angle turnPosition = Radians.of(0.0);
        public Angle turnEncoderPosition = Radians.of(0.0);
        public Angle turnSetpoint = Radians.of(0.0);
        public AngularVelocity turnVelocity = RadiansPerSecond.of(0.0);

        public boolean hoodMotorConnected = false;
        public Voltage hoodAppliedVolts = Volts.of(0.0);
        public Current hoodCurrent = Amps.of(0.0);
        public Current hoodSupplyCurrent = Amps.of(0.0);
        public Angle hoodPosition = Radians.of(0.0);
        public Angle hoodSetpoint = Radians.of(0.0);
        public AngularVelocity hoodVelocity = RadiansPerSecond.of(0.0);

        public boolean flywheelMotorConnected = false;
        public Voltage flywheelAppliedVolts = Volts.of(0.0);
        public Current flywheelCurrent = Amps.of(0.0);
        public Current flywheelSupplyCurrent = Amps.of(0.0);
        public Current flywheelLeaderCurrent = Amps.of(0.0);
        public Current flywheelLeaderSupplyCurrent = Amps.of(0.0);
        public Current flywheelFollowerCurrent = Amps.of(0.0);
        public Current flywheelFollowerSupplyCurrent = Amps.of(0.0);
        public AngularVelocity flywheelSpeed = RadiansPerSecond.of(0.0);
        public AngularAcceleration flywheelAccel = RadiansPerSecondPerSecond.of(0.0);
        public AngularVelocity flywheelSetpointSpeed = RadiansPerSecond.of(0.0);
        public AngularAcceleration flywheelSetpointAccel = RadiansPerSecondPerSecond.of(0.0);
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void setTurnSetpoint(Angle position, AngularVelocity velocity) {}

    public default void setHoodAngle(Angle angle) {}

    public default void setHoodOut(Voltage out) {}

    public default void setFlywheelSpeed(AngularVelocity speed) {}

    public default void stopTurn() {}

    public default void stopHood() {}

    public default void stopFlywheel() {}

    public default void resetTurnEncoder() {}

    public default void zeroHoodPosition() {}

    public default void setTurnPID(double kP, double kD, double kV, double kS) {}

    public default void setHoodPID(double kP, double kD, double kS) {}

    public default void setFlywheelPID(double kP, double kD, double kV, double kS) {}
}
