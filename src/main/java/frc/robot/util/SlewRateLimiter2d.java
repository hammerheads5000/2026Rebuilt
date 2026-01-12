// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotController;

/**
 * Wrapper class for {@link edu.wpi.first.math.MathUtil#slewRateLimit} to mimic behavior of
 * {@link edu.wpi.first.math.filter.SlewRateLimiter}
 */
public class SlewRateLimiter2d {
    private double rateLimit;

    private Translation2d prevTranslation; // in terms of base unit
    private Time prevTime;

    /**
     * Creates a new SlewRateLimiter2d with the specified rate limit.
     *
     * @param rateLimit The maximum rate of change. (Positive)
     */
    public SlewRateLimiter2d(double rateLimit) {
        this.rateLimit = rateLimit;

        prevTranslation = new Translation2d();
        prevTime = Seconds.zero();
    }

    /**
     * Calculates the new value based on the previous value and the rate limit.
     *
     * @param translation the translation to be limited
     * @return the translation limited by the rate limit in terms of magnitude
     */
    public Translation2d calculate(Translation2d translation) {
        Time currentTime = RobotController.getMeasureTime();
        double deltaTime = currentTime.minus(prevTime).in(Seconds);
        prevTime = currentTime;

        prevTranslation = MathUtil.slewRateLimit(prevTranslation, translation, deltaTime, rateLimit);

        return prevTranslation;
    }

    /**
     * Calculates the new value based on the previous value and the rate limit.
     *
     * @param x the x component to be limited
     * @param y the y component to be limited
     * @return a translation of the x and y limited by the rate limit in terms of magnitude
     */
    public Translation2d calculate(double x, double y) {
        return calculate(new Translation2d(x, y));
    }

    /**
     * Resets the slew rate limiter to the specified translation, ignoring rate limit.
     *
     * @param translation the translation to reset to
     */
    public void reset(Translation2d translation) {
        prevTranslation = translation;
        prevTime = RobotController.getMeasureTime();
    }

    public void setRateLimit(double rateLimit) {
        this.rateLimit = rateLimit;
    }
}
