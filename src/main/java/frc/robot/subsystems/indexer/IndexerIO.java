package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public boolean spinMotorConnected = false;
        public AngularVelocity spinVelocity = RadiansPerSecond.of(0.0);
        public Current spinCurrent = Amps.of(0.0);
        public Current spinSupplyCurrent = Amps.of(0.0);
        public Voltage spinAppliedVolts = Volts.of(0.0);
        public Temperature spinTemp = Celsius.of(0);

        public boolean feedMotorConnected = false;
        public AngularVelocity feedVelocity = RadiansPerSecond.of(0.0);
        public Current feedCurrent = Amps.of(0.0);
        public Current feedSupplyCurrent = Amps.of(0.0);
        public Voltage feedAppliedVolts = Volts.of(0.0);
        public Temperature feedTemp = Celsius.of(0);
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void setSpinOutput(Voltage out) {}

    public default void setFeedOutput(Voltage out) {}

    public default void stopSpin() {}

    public default void stopFeed() {}
}
