package frc.robot.subsystems;

public interface ClimbIO {
    // Motor control
    void setWinchVoltage(double volts);
    default void setArmVoltage(double volts) {}
    void stopAll();

    // Position control (optional)
    default void setWinchPositionMeters(double meters) {}
    default double getWinchPositionMeters() { return 0.0; }

    default void setArmAngleRadians(double radians) {}
    default double getArmAngleRadians() { return 0.0; }

    // Sensors
    default boolean atLowerLimit() { return false; }
    default boolean atUpperLimit() { return false; }
    default boolean hookEngaged() { return false; }
    default double getWinchCurrent() { return 0.0; }

    // Locking / ratchet (optional)
    default void setRatchetEngaged(boolean engaged) {}
}