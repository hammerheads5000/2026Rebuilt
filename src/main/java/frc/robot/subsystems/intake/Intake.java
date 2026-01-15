// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.TunableControls.TunableProfiledController;
import java.util.function.Supplier;

public class Intake extends SubsystemBase {
    private final IntakeIO leftIO;
    private final IntakeIO rightIO;
    private final IntakeIOInputsAutoLogged leftInputs;
    private final IntakeIOInputsAutoLogged rightInputs;

    private final TunableProfiledController leftController;
    private final TunableProfiledController rightController;

    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    private boolean isDeployed = false;
    private Trigger deployLeftTrigger =
            new Trigger(this::travelingLeft).and(() -> isDeployed).debounce(0.2);
    private Trigger deployRightTrigger =
            new Trigger(this::travelingRight).and(() -> isDeployed).debounce(0.2);

    /** Creates a new Intake. */
    public Intake(IntakeIO leftIO, IntakeIO rightIO, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.leftIO = leftIO;
        this.rightIO = rightIO;

        this.leftInputs = new IntakeIOInputsAutoLogged();
        this.rightInputs = new IntakeIOInputsAutoLogged();

        this.leftController = new TunableProfiledController(RACK_TUNABLE_CONSTANTS);
        this.rightController = new TunableProfiledController(RACK_TUNABLE_CONSTANTS);

        this.chassisSpeedsSupplier = chassisSpeedsSupplier;

        this.deployLeftTrigger.onTrue(deployLeft());
        this.deployRightTrigger.onTrue(deployRight());
    }

    private boolean travelingLeft() {
        return chassisSpeedsSupplier.get().vyMetersPerSecond > MIN_SWITCH_ROBOT_VELOCITY.in(MetersPerSecond);
    }

    private boolean travelingRight() {
        return chassisSpeedsSupplier.get().vyMetersPerSecond < -MIN_SWITCH_ROBOT_VELOCITY.in(MetersPerSecond);
    }

    public Command deploy() {
        return this.runOnce(() -> isDeployed = true)
                .andThen(deployLeft().onlyIf(() -> !travelingLeft() && !travelingRight()));
    }

    private Command deployLeft() {
        return this.runOnce(() -> {
            leftController.setGoal(DEPLOY_POS.in(Meters));
            rightController.setGoal(STOW_POS.in(Meters));

            leftIO.setSpinOutput(SPIN_VOLTAGE);
            rightIO.stopSpin();
        });
    }

    private Command deployRight() {
        return this.runOnce(() -> {
            leftController.setGoal(STOW_POS.in(Meters));
            rightController.setGoal(DEPLOY_POS.in(Meters));

            leftIO.stopSpin();
            rightIO.setSpinOutput(SPIN_VOLTAGE);
        });
    }

    public Command stow() {
        return this.runOnce(() -> {
            isDeployed = false;
            leftController.setGoal(STOW_POS.in(Meters));
            rightController.setGoal(STOW_POS.in(Meters));

            leftIO.stopSpin();
            rightIO.stopSpin();
        });
    }

    @Override
    public void periodic() {
        leftIO.updateInputs(leftInputs);
        rightIO.updateInputs(rightInputs);

        leftIO.setRackOutput(Volts.of(leftController.calculate(leftInputs.rackPosition.in(Meters))));
        rightIO.setRackOutput(Volts.of(rightController.calculate(rightInputs.rackPosition.in(Meters))));
    }
}
