package frc.robot.subsystems.intake;

// Interface for Intake IO implementations (hardware or simulation)

public interface IntakeIO {
    // Roller control
    void setRollerVoltage(double volts);
    void stopRoller();

    // Deployment (optional)
    // Set deploy motor open-loop voltage (positive deploy, negative stow)
    default void setDeployVoltage(double volts) {}

    // Stop the deploy motor
    default void stopDeployMotor() {}

    // Metrics for deploy monitoring
    default double getDeployCurrent() { return 0.0; } // amps
    default double getDeployVelocityRadPerSec() { return 0.0; }

    // Sensors
    default boolean hasGamePiece() { return false; }
    default double getRollerCurrent() { return 0.0; }
}
