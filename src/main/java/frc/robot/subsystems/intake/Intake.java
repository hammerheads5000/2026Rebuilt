// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO leftIO;
    private final IntakeIO rightIO;
    private final IntakeIOInputsAutoLogged leftInputs;
    private final IntakeIOInputsAutoLogged rightInputs;

    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    private boolean isDeployed = true;
    private boolean leftDeployed = false;
    private boolean rightDeployed = false;
    public Trigger deployLeftTrigger =
            new Trigger(this::travelingLeft).and(() -> isDeployed).debounce(0.2);
    public Trigger deployRightTrigger =
            new Trigger(this::travelingRight).and(() -> isDeployed).debounce(0.2);

    private final IntakeVisualizer measuredVisualizer = new IntakeVisualizer("Measured", Color.kGreen);

    /** Creates a new Intake. */
    public Intake(IntakeIO leftIO, IntakeIO rightIO, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.leftIO = leftIO;
        this.rightIO = rightIO;

        this.leftInputs = new IntakeIOInputsAutoLogged();
        this.rightInputs = new IntakeIOInputsAutoLogged();

        this.chassisSpeedsSupplier = chassisSpeedsSupplier;

        this.deployLeftTrigger.onTrue(deployLeft());
        this.deployRightTrigger.onTrue(deployRight());

        SmartDashboard.putData(deployLeft());
        SmartDashboard.putData(deployRight());
        SmartDashboard.putData(stow());
    }

    private boolean travelingLeft() {
        return chassisSpeedsSupplier.get().vyMetersPerSecond > MIN_SWITCH_ROBOT_VELOCITY.in(MetersPerSecond);
    }

    private boolean travelingRight() {
        return chassisSpeedsSupplier.get().vyMetersPerSecond < -MIN_SWITCH_ROBOT_VELOCITY.in(MetersPerSecond);
    }

    public boolean isLeftDeployed() {
        return leftDeployed;
    }

    public boolean isRightDeployed() {
        return rightDeployed;
    }

    public Command deploy() {
        return this.runOnce(() -> isDeployed = true)
                .andThen(deployLeft().onlyIf(() -> !travelingLeft() && !travelingRight()))
                .withName("Deploy intake");
    }

    private Command deployLeft() {
        return this.runOnce(() -> {
                    leftIO.setRackPosition(DEPLOY_POS);
                    rightIO.setRackPosition(STOW_POS);

                    leftIO.setSpinOutput(SPIN_VOLTAGE);
                    rightIO.stopSpin();
                })
                .withName("Deploy Left Intake");
    }

    private Command deployRight() {
        return this.runOnce(() -> {
                    leftIO.setRackPosition(STOW_POS);
                    rightIO.setRackPosition(DEPLOY_POS);

                    leftIO.stopSpin();
                    rightIO.setSpinOutput(SPIN_VOLTAGE);
                })
                .withName("Deploy Right Intake");
    }

    public Command stow() {
        return this.runOnce(() -> {
                    isDeployed = false;
                    leftIO.setRackPosition(STOW_POS);
                    rightIO.setRackPosition(STOW_POS);

                    leftIO.stopSpin();
                    rightIO.stopSpin();
                })
                .withName("Stow intakes");
    }

    @Override
    public void periodic() {
        leftIO.updateInputs(leftInputs);
        rightIO.updateInputs(rightInputs);

        Logger.processInputs("Left Intake", leftInputs);
        Logger.processInputs("Right Intake", rightInputs);

        measuredVisualizer.setLeftPosition(leftInputs.rackPosition);
        measuredVisualizer.setRightPosition(rightInputs.rackPosition);
    }
}
