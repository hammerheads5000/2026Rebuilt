// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.SwerveConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {
    // TunerConstants doesn't support separate sim constants, so they are declared
    // locally
    private static final double DRIVE_KP = 0.05;
    private static final double DRIVE_KD = 0.0;
    private static final double DRIVE_KS = 0.0;
    private static final double DRIVE_KV_ROT = 0.91035; // Same units as TunerConstants: (volt * secs) / rotation
    private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
    private static final double TURN_KP = 8.0;
    private static final double TURN_KD = 0.0;

    private final SwerveModuleSimulation moduleSimulation;
    private final SimulatedMotorController.GenericMotorController driveSim, turnSim;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private PIDController driveController = new PIDController(DRIVE_KP, 0, DRIVE_KD);
    private PIDController turnController = new PIDController(TURN_KP, 0, TURN_KD);
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;
    private double desiredMotorVelocityRadPerSec = 0.0;
    private Rotation2d desiredSteerFacing = Rotation2d.kZero;

    public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
        // Create drive and turn sim models
        this.moduleSimulation = moduleSimulation;
        driveSim = moduleSimulation
                .useGenericMotorControllerForDrive()
                .withCurrentLimit(SwerveConstants.DRIVE_CONFIGS.CurrentLimits.getStatorCurrentLimitMeasure());
        turnSim = moduleSimulation
                .useGenericControllerForSteer()
                .withCurrentLimit(SwerveConstants.STEER_CONFIGS.CurrentLimits.getStatorCurrentLimitMeasure());

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);
        SimulatedArena.getInstance().addCustomSimulation((subTickNum) -> runControlLoops());
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update drive inputs
        inputs.driveConnected = true;
        inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
        inputs.driveVelocityRadPerSec =
                moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = moduleSimulation.getDriveMotorStatorCurrent().abs(Amps);

        // Update turn inputs
        inputs.turnConnected = true;
        inputs.turnEncoderConnected = true;
        inputs.turnAbsolutePosition = moduleSimulation.getSteerAbsoluteFacing();
        inputs.turnPosition = moduleSimulation.getSteerAbsoluteFacing();
        inputs.turnVelocityRadPerSec =
                moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = moduleSimulation.getSteerMotorStatorCurrent().abs(Amps);

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
        // matter)
        inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
        inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
        inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
    }

    public void runControlLoops() {
        // Run control loops if activated
        if (driveClosedLoop) calculateDriveControlLoops();
        else driveController.reset();
        if (turnClosedLoop) calculateSteerControlLoops();
        else turnController.reset();

        // Feed voltage to motor simulation
        driveSim.requestVoltage(Volts.of(driveAppliedVolts));
        turnSim.requestVoltage(Volts.of(turnAppliedVolts));
    }

    private void calculateDriveControlLoops() {
        DCMotor motorModel = moduleSimulation.config.driveMotorConfigs.motor;
        double frictionTorque =
                motorModel.getTorque(motorModel.getCurrent(0, SwerveConstants.DRIVE_FRICTION_VOLTAGE.in(Volts)))
                        * Math.signum(desiredMotorVelocityRadPerSec);
        double velocityFeedforwardVolts = motorModel.getVoltage(frictionTorque, desiredMotorVelocityRadPerSec);
        double feedforwardVolts = velocityFeedforwardVolts;
        double feedBackVolts = driveController.calculate(
                moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond),
                desiredMotorVelocityRadPerSec / SwerveConstants.DRIVE_GEAR_RATIO);
        driveAppliedVolts = feedforwardVolts + feedBackVolts;
    }

    private void calculateSteerControlLoops() {
        turnAppliedVolts = turnController.calculate(
                moduleSimulation.getSteerAbsoluteFacing().getRadians(), desiredSteerFacing.getRadians());
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveClosedLoop = false;
        driveAppliedVolts = output;
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnClosedLoop = false;
        turnAppliedVolts = output;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveClosedLoop = true;
        desiredMotorVelocityRadPerSec = velocityRadPerSec;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        desiredSteerFacing = rotation;
        turnController.setSetpoint(rotation.getRadians());
    }
}
