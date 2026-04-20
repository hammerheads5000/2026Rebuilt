package frc.robot.util;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Joules;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Energy;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VirtualPD {
    private static ArrayList<Supplier<Current>> motors = new ArrayList<>();
    private static ArrayList<String> groups = new ArrayList<>();
    private static HashMap<String, Energy> groupEnergyTotals = new HashMap<>();
    private static ArrayList<Energy> motorEnergyTotals = new ArrayList<>();
    private static Energy totalEnergy = Joules.zero();

    public static void registerMotor(Supplier<Current> currentSupplier, String group) {
        motors.add(currentSupplier);
        groups.add(group);
        motorEnergyTotals.add(Joules.zero().mutableCopy());
        if (!groupEnergyTotals.containsKey(group)) {
            groupEnergyTotals.put(group, Joules.zero());
        }
    }

    public static void logTotalCurrent() {
        MutCurrent total = Amps.zero().mutableCopy();
        HashMap<String, Current> groupCurrentTotals = new HashMap<>();
        Voltage batteryVoltage = RobotController.getMeasureBatteryVoltage();

        for (int i = 0; i < motors.size(); i++) {
            Current current = motors.get(i).get();
            Power power = current.times(batteryVoltage);
            Energy energy = power.times(Seconds.of(0.02));
            motorEnergyTotals.set(i, motorEnergyTotals.get(i).plus(energy));
            total.mut_plus(current);
            totalEnergy = totalEnergy.plus(energy);
            String group = groups.get(i);
            if (groupCurrentTotals.containsKey(group)) {
                groupCurrentTotals.put(group, groupCurrentTotals.get(group).plus(current));
            } else {
                groupCurrentTotals.put(group, current);
            }
            groupEnergyTotals.put(group, groupEnergyTotals.get(group).plus(energy));
        }

        Logger.recordOutput("VirtualPD/Total", total);
        for (String group : groupCurrentTotals.keySet()) {
            Logger.recordOutput("VirtualPD/Current/" + group, groupCurrentTotals.get(group));
        }
        for (String group : groupEnergyTotals.keySet()) {
            Logger.recordOutput("VirtualPD/Energy/" + group, groupEnergyTotals.get(group));
        }
    }
}
