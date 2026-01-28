// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;

public class IntakeIOSim implements IntakeIO {
    private ProfiledPIDController rackProfile =
            new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(10, 10));

    /** Creates a new IntakeIOSim. */
    public IntakeIOSim() {}

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rackMotorConnected = true;
        inputs.rackPosition = Meters.of(rackProfile.getSetpoint().position);
        inputs.rackVelocity = MetersPerSecond.of(rackProfile.getSetpoint().velocity);
        inputs.rackSetpoint = Meters.of(rackProfile.getSetpoint().position);
        inputs.rackSetpointVelocity = MetersPerSecond.of(rackProfile.getSetpoint().velocity);
    }

    @Override
    public void setRackPosition(Distance position) {
        rackProfile.setGoal(position.in(Meters));
    }
}
