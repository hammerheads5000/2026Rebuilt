package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.HubShiftUtil.ShiftEnum;
import frc.robot.util.HubShiftUtil.ShiftInfo;

public class Leds {
    private static final int START = 8; // front of right
    private static final int END_RIGHT = 81; // back of right
    private static final int END = 161; // back of left

    private static final SolidColor BLANK = new SolidColor(START, END).withColor(new RGBWColor());
    private static final RainbowAnimation RAINBOW =
            new RainbowAnimation(START, END).withBrightness(0.8).withFrameRate(30);
    private static final LarsonAnimation CYLON =
            new LarsonAnimation(START, END).withBounceMode(LarsonBounceValue.Center).withColor(new RGBWColor(Color.kRed)).withSize(15).withFrameRate(60);

    private RGBWColor activeColor = new RGBWColor(Color.kGreen);
    private RGBWColor inactiveColor = new RGBWColor(Color.kRed);
    private RGBWColor endgameColor = new RGBWColor(Color.kPurple);

    private final CANdle candle;

    @AutoLogOutput
    private LEDMode currentMode = LEDMode.RAINBOW;

    private final Trigger disabledTrigger = new Trigger(DriverStation::isDisabled);
    private final Trigger teleopTrigger = new Trigger(DriverStation::isTeleopEnabled);
    private final Trigger autoTrigger = new Trigger(DriverStation::isAutonomousEnabled);

    public Leds() {
        this.candle = new CANdle(7);

        teleopTrigger.onTrue(Commands.runOnce(() -> this.setMode(LEDMode.SHIFTS)).withName("Set LED Mode Shifts"));
        disabledTrigger.onTrue(Commands.runOnce(() -> this.setMode(LEDMode.RAINBOW)).withName("Set LED Mode Rainbow"));
        autoTrigger.onTrue(Commands.runOnce(() -> this.setMode(LEDMode.CYLON)).withName("Set LED Mode Cylon"));
    }

    public void setMode(LEDMode mode) {
        currentMode = mode;
        candle.clearAllAnimations();
        switch (mode) {
            case OFF:
                candle.setControl(BLANK);
                break;
            case SHIFTS:
                candle.setControl(BLANK);

                break;
            case RAINBOW:
                candle.setControl(RAINBOW);
                break;
            case CYLON:
                candle.setControl(CYLON.withSlot(0).withLEDStartIndex(START).withLEDEndIndex(END_RIGHT));
                candle.setControl(CYLON.withSlot(1).withLEDStartIndex(END_RIGHT + 1).withLEDEndIndex(END));
                break;
        }
    }

    public void periodic() {
        if (currentMode != LEDMode.SHIFTS) return;

        ShiftInfo shiftInfo = HubShiftUtil.getOfficialShiftInfo();
        RGBWColor color = shiftInfo.currentShift() == ShiftEnum.ENDGAME ? endgameColor : (shiftInfo.active() ? activeColor : inactiveColor);
        double proportion = shiftInfo.elapsedTime() / (shiftInfo.remainingTime() + shiftInfo.elapsedTime());

        int rightPixel = (int) (proportion * (END_RIGHT - START)) + START;
        int leftPixel = (int) (proportion * (END - END_RIGHT - 1)) + END_RIGHT + 1;
        
        candle.setControl(BLANK.withLEDStartIndex(rightPixel + 1).withLEDEndIndex(END_RIGHT));
        candle.setControl(BLANK.withLEDStartIndex(leftPixel + 1).withLEDEndIndex(END));

        if (shiftInfo.remainingTime() <= 5.0) {
            int frameRate = shiftInfo.remainingTime() <= 3.0 ? 200 : 100;
            candle.setControl(new SingleFadeAnimation(START, rightPixel).withColor(color).withFrameRate(frameRate).withSlot(0));
            candle.setControl(new SingleFadeAnimation(END_RIGHT + 1, leftPixel).withColor(color).withFrameRate(frameRate).withSlot(1));
        } else {
            candle.setControl(new SolidColor(START, rightPixel).withColor(color));
            candle.setControl(new SolidColor(END_RIGHT + 1, leftPixel).withColor(color));
        }
    }

    public enum LEDMode {
        OFF, SHIFTS, RAINBOW, CYLON
    }
}
