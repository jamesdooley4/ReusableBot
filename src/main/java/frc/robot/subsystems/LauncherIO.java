package frc.robot.subsystems;

public interface LauncherIO {
    // Flywheel control
    void setFlywheelVoltage(double volts);
    default void setFlywheelVelocity(double radPerSec) {}
    void stopFlywheel();

    // Optional second wheel / stage
    default void setTopFlywheelVoltage(double volts) {}
    default void setTopFlywheelVelocity(double radPerSec) {}
    default void stopTopFlywheel() {}

    // Hood / angle control (optional)
    default void setHoodPositionRadians(double radians) {}
    default double getHoodPositionRadians() { return 0.0; }

    // Feedback
    default double getFlywheelVelocityRadPerSec() { return 0.0; }
    default double getTopFlywheelVelocityRadPerSec() { return 0.0; }
    default double getFlywheelCurrent() { return 0.0; }

    // Diagnostics
    default boolean isHomed() { return true; }
}
