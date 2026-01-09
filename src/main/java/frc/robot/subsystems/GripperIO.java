package frc.robot.subsystems;

public interface GripperIO {
    // Roller or motorized gripper
    default void setRollerVoltage(double volts) {}
    default void stopRoller() {}

    // Pneumatic or latch‑style gripper
    default void setOpen(boolean open) {}

    // Sensors
    default boolean hasGamePiece() { return false; }
    default double getRollerCurrent() { return 0.0; }
}
