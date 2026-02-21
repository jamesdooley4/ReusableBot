package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public interface TurretIO {
    // Simulation
    public SingleJointedArmSim getSimulation();

    // Control
    void setVoltage(double volts);
    void stop();

    // Position control (if supported)
    default void setPositionRadians(double radians) {}
    default void setVelocityRadiansPerSec(double radPerSec) {}

    // Feedback
    default double getPositionRadians() { return 0.0; }
    default double getVelocityRadiansPerSec() { return 0.0; }
    default double getSupplyCurrent() { return 0.0; }

    // Limits / homing
    default boolean getForwardLimitSwitch() { return false; }
    default boolean getReverseLimitSwitch() { return false; }
    default boolean isHomed() { return true; }
}
