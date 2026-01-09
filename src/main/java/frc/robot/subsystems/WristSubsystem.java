package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class WristSubsystem extends SubsystemBase {
    public enum State {
        IDLE,
        MOVING_TO_ANGLE,
        MANUAL
    }

    private final WristIO io;
    private State state = State.IDLE;
    private double targetAngle = 0.0;

    public WristSubsystem(WristIO io) {
        this.io = io;
    }

    public void moveToAngle(double radians) {
        state = State.MOVING_TO_ANGLE;
        targetAngle = radians;
        io.setWristAngleRadians(radians);
    }

    public void manualControl(double volts) {
        state = State.MANUAL;
        io.setWristVoltage(volts);
    }

    public boolean atAngle(double tolerance) {
        return Math.abs(io.getWristAngleRadians() - targetAngle) <= tolerance;
    }

    public void stop() {
        state = State.IDLE;
        io.setWristVoltage(0.0);
    }

    public State getState() {
        return state;
    }

    // --- Command factories ---

    /** Move wrist to an angle and finish when within tolerance. */
    public Command moveToAngleCommand(double radians, double tolerance) {
        return Commands.sequence(
            runOnce(() -> moveToAngle(radians)),
            run(() -> {}).until(() -> atAngle(tolerance)),
            runOnce(() -> stop())
        );
    }

    /** Manual control command that reads a single supplier for volts. */
    public Command manualControlCommand(DoubleSupplier voltsSupplier) {
        return run(() -> manualControl(voltsSupplier.getAsDouble()));
    }
}
