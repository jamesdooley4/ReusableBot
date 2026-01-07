package frc.robot.subsystems;

public interface IntakeIO {
    // Roller control
    void setRollerVoltage(double volts);
    void stopRoller();

    // Deployment (optional)
    default void setDeployed(boolean deployed) {}
    default boolean isDeployed() { return true; }

    // Sensors
    default boolean hasGamePiece() { return false; }
    default double getRollerCurrent() { return 0.0; }
}