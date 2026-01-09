package frc.robot.subsystems;

public interface IndexIO {
    // Motor control
    void setConveyorVoltage(double volts);
    void stopConveyor();

    // Optional multi‑stage indexers
    default void setStage2Voltage(double volts) {}
    default void stopStage2() {}

    // Sensors
    default boolean entrySensorTriggered() { return false; }
    default boolean midSensorTriggered() { return false; }
    default boolean exitSensorTriggered() { return false; }

    // Diagnostics
    default double getConveyorCurrent() { return 0.0; }
}