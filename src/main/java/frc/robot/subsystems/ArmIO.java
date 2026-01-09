package frc.robot.subsystems;

public interface ArmIO {
    // Control
    void setShoulderVoltage(double volts);
    default void setElbowVoltage(double volts) {}

    // Position control (optional)
    default void setShoulderAngleRadians(double radians) {}
    default void setElbowAngleRadians(double radians) {}

    // Feedback
    default double getShoulderAngleRadians() { return 0.0; }
    default double getElbowAngleRadians() { return 0.0; }

    // Diagnostics
    default double getShoulderCurrent() { return 0.0; }
    default double getElbowCurrent() { return 0.0; }

    // Limits / homing
    default boolean shoulderAtLowerLimit() { return false; }
    default boolean shoulderAtUpperLimit() { return false; }
    default boolean elbowAtLowerLimit() { return false; }
    default boolean elbowAtUpperLimit() { return false; }
}
