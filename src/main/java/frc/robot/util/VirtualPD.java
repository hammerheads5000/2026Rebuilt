package frc.robot.util;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutCurrent;
import java.util.ArrayList;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VirtualPD {
    private static ArrayList<Supplier<Current>> motors = new ArrayList<>();

    public static void registerMotor(Supplier<Current> currentSupplier) {
        motors.add(currentSupplier);
    }

    public static void logTotalCurrent() {
        MutCurrent total = Amps.zero().mutableCopy();

        for (Supplier<Current> motor : motors) {
            total.mut_plus(motor.get());
        }

        Logger.recordOutput("VirtualPD", total);
    }
}
