// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public class TurretIOSim implements TurretIO {
    private Angle turnPosition = Radians.zero();
    private Angle hoodAngle = Radians.zero();
    private AngularVelocity flywheelGoal = RadiansPerSecond.zero();

    private SlewRateLimiter flywheelAccelLimiter =
            new SlewRateLimiter(RPM.of(4000).div(Seconds.of(4)).in(RadiansPerSecondPerSecond));

    public TurretIOSim() {}

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.turnPosition = turnPosition;
        inputs.hoodPosition = hoodAngle;
        inputs.flywheelSpeed = RadiansPerSecond.of(flywheelAccelLimiter.calculate(flywheelGoal.in(RadiansPerSecond)));
    }

    @Override
    public void setTurnSetpoint(Angle position, AngularVelocity velocity) {
        turnPosition = position;
    }

    @Override
    public void setHoodAngle(Angle angle) {
        hoodAngle = angle;
    }

    @Override
    public void setFlywheelSpeed(AngularVelocity speed) {
        flywheelGoal = speed;
    }

    @Override
    public void stopFlywheel() {
        flywheelGoal = RadiansPerSecond.zero();
    }
}
