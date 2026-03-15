package frc.robot.util;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutCurrent;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VirtualPD {
    private static ArrayList<Supplier<Current>> motors = new ArrayList<>();
    private static ArrayList<String> groups = new ArrayList<>();

    public static void registerMotor(Supplier<Current> currentSupplier, String group) {
        motors.add(currentSupplier);
        groups.add(group);
    }

    public static void logTotalCurrent() {
        MutCurrent total = Amps.zero().mutableCopy();
        HashMap<String, Current> groupTotals = new HashMap<>();

        for (int i = 0; i < motors.size(); i++) {
            Current current = motors.get(i).get();
            total.mut_plus(current);
            String group = groups.get(i);
            if (groupTotals.containsKey(group)) {
                groupTotals.put(group, groupTotals.get(group).plus(current));
            } else {
                groupTotals.put(group, current);
            }
        }

        Logger.recordOutput("VirtualPD/Total", total);
        for (String group : groupTotals.keySet()) {
            Logger.recordOutput("VirtualPD/" + group, groupTotals.get(group));
        }
    }
}
