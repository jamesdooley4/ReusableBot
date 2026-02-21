package frc.robot.subsystems.wrist;

public interface WristIO {
    void setWristVoltage(double volts);
    default void setWristAngleRadians(double radians) {}

    default double getWristAngleRadians() { return 0.0; }
    default double getWristCurrent() { return 0.0; }

    default boolean atLowerLimit() { return false; }
    default boolean atUpperLimit() { return false; }
}
