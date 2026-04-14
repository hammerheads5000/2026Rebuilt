package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.HubShiftUtil.ShiftEnum;
import frc.robot.util.HubShiftUtil.ShiftInfo;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Leds {
    private static final int START = 8; // front of right
    private static final int END_RIGHT = 85; // back of right
    private static final int END = 161; // back of left
    private static final int ENDGAME_RIGHT_PIX = (int) (30.0 / 55.0 * (END_RIGHT - START)) + START;
    private static final int ENDGAME_LEFT_PIX = (int) (30.0 / 55.0 * (END - END_RIGHT - 1)) + END_RIGHT + 1;
    private static final double BRIGHTNESS = 0.6;

    private static final SolidColor BLANK = new SolidColor(START, END).withColor(new RGBWColor());
    private static final SolidColor WHITE =
            new SolidColor(START, END).withColor(new RGBWColor(Color.kWhite).scaleBrightness(BRIGHTNESS / 3.0));
    private static final SingleFadeAnimation LOW_BATTERY = new SingleFadeAnimation(START, END)
            .withFrameRate(250)
            .withColor(new RGBWColor(Color.kRed).scaleBrightness(BRIGHTNESS));
    private static final RainbowAnimation RAINBOW =
            new RainbowAnimation(START, END).withBrightness(BRIGHTNESS).withFrameRate(30);
    private static final LarsonAnimation CYLON = new LarsonAnimation(START, END)
            .withBounceMode(LarsonBounceValue.Center)
            .withColor(new RGBWColor(Color.kRed).scaleBrightness(BRIGHTNESS))
            .withSize(15)
            .withFrameRate(50);

    private RGBWColor activeColor = new RGBWColor(Color.kBlue).scaleBrightness(BRIGHTNESS);
    private RGBWColor inactiveColor = new RGBWColor(Color.kRed).scaleBrightness(BRIGHTNESS);
    private RGBWColor endgameColor = new RGBWColor(Color.kPurple).scaleBrightness(BRIGHTNESS);

    private final CANdle candle;

    @AutoLogOutput
    private LEDMode currentMode = LEDMode.RAINBOW;

    private boolean last5Secs = false;

    private final BooleanSupplier disabledTrigger = DriverStation::isDisabled;
    private final BooleanSupplier teleopTrigger = DriverStation::isTeleopEnabled;
    private final BooleanSupplier autoTrigger = DriverStation::isAutonomousEnabled;
    private final BooleanSupplier lowBatteryTrigger = () -> RobotController.getBatteryVoltage() <= 12.0;
    private final Debouncer lowBatteryDebouncer = new Debouncer(5);

    public Leds() {
        this.candle = new CANdle(7, Constants.CAN_FD_BUS);

        candle.clearAllAnimations();
        candle.setControl(RAINBOW);
    }

    public void setMode(LEDMode mode) {
        currentMode = mode;
        candle.clearAllAnimations();
        switch (mode) {
            case OFF:
                candle.setControl(BLANK);
                break;
            case TELEOP:
                candle.setControl(BLANK);
                break;
            case RAINBOW:
                candle.setControl(RAINBOW);
                System.out.println("hidfhsiheihs");
                break;
            case AUTO:
                candle.setControl(CYLON.withSlot(0).withLEDStartIndex(START).withLEDEndIndex(END_RIGHT));
                candle.setControl(
                        CYLON.withSlot(1).withLEDStartIndex(END_RIGHT + 1).withLEDEndIndex(END));
                break;
            case LOW_BATTERY:
                candle.setControl(LOW_BATTERY);
                break;
        }
    }

    public void periodic() {
        if (teleopTrigger.getAsBoolean() && currentMode != LEDMode.TELEOP) {
            setMode(LEDMode.TELEOP);
        } else if (autoTrigger.getAsBoolean() && currentMode != LEDMode.AUTO) {
            setMode(LEDMode.AUTO);
        } else if (disabledTrigger.getAsBoolean()) {
            boolean lowBattery = lowBatteryDebouncer.calculate(lowBatteryTrigger.getAsBoolean());
            if (lowBattery && currentMode != LEDMode.LOW_BATTERY) {
                setMode(LEDMode.LOW_BATTERY);
            } else if (!lowBattery && currentMode != LEDMode.RAINBOW) {
                setMode(LEDMode.RAINBOW);
            }
        }

        if (currentMode != LEDMode.TELEOP) return;

        ShiftInfo shiftInfo = HubShiftUtil.getOfficialShiftInfo();

        // if (prevShift != shiftInfo.currentShift()) {
        //     prevShift = shiftInfo.currentShift();
        //     candle.clearAllAnimations();
        //     candle.setControl(BLANK);
        // }

        RGBWColor color = shiftInfo.currentShift() == ShiftEnum.ENDGAME
                ? endgameColor
                : (shiftInfo.active() ? activeColor : inactiveColor);
        RGBWColor offColor = new RGBWColor(Color.kWhite);
        double proportion = shiftInfo.remainingTime() / (shiftInfo.remainingTime() + shiftInfo.elapsedTime());

        // candle.clearAllAnimations();

        if (shiftInfo.remainingTime() <= 5.0) {
            // color = shiftInfo.currentShift() == ShiftEnum.SHIFT4
            //         ? endgameColor
            //         : (shiftInfo.active() ? inactiveColor : activeColor);
            Frequency frameRate = Hertz.of(1);
            // // Frequency frameRate = Hertz.of(shiftInfo.remainingTime() <= 3.0 ? 2 : 1);
            // proportion = (5.0 - shiftInfo.remainingTime()) / 5.0;
            color = applyFade(color, frameRate, shiftInfo.remainingTime());
            offColor = applyFade(offColor, frameRate, shiftInfo.remainingTime());
        }

        int rightPixel = (int) (proportion * (END_RIGHT - START)) + START;
        int leftPixel = (int) (proportion * (END - END_RIGHT - 1)) + END_RIGHT + 1;

        candle.setControl(new SolidColor(rightPixel + 1, END_RIGHT).withColor(offColor));
        candle.setControl(new SolidColor(leftPixel + 1, END).withColor(offColor));

        if (shiftInfo.currentShift() == ShiftEnum.SHIFT4 && shiftInfo.active()) {
            candle.setControl(new SolidColor(ENDGAME_RIGHT_PIX + 1, rightPixel).withColor(color));
            candle.setControl(new SolidColor(ENDGAME_LEFT_PIX + 1, leftPixel).withColor(color));

            candle.setControl(new SolidColor(START, ENDGAME_RIGHT_PIX).withColor(endgameColor));
            candle.setControl(new SolidColor(END_RIGHT + 1, ENDGAME_LEFT_PIX).withColor(endgameColor));
        } else {
            candle.setControl(new SolidColor(START, rightPixel).withColor(color));
            candle.setControl(new SolidColor(END_RIGHT + 1, leftPixel).withColor(color));
        }
    }

    private static RGBWColor applyFade(RGBWColor color, Frequency freq, double t) {
        double scale = Math.sin(2 * Math.PI * t * freq.in(Hertz));
        scale = scale * 0.5 + 0.5;
        return color.scaleBrightness(scale);
    }

    public enum LEDMode {
        OFF,
        TELEOP,
        RAINBOW,
        LOW_BATTERY,
        AUTO
    }
}
