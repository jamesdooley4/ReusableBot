package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

public class TurretSubsystem extends SubsystemBase {
    public interface TurretTargetProvider {
        // Angle offset from current turret angle to target, in radians.
        // Positive = turn CCW, negative = turn CW (or vice versa, but consistent).
        boolean hasTarget();

        double getTargetOffsetRadians();
    }

    public enum State {
        IDLE,
        MANUAL,
        SEEKING,
        TRACKING,
        MOVING_TO_ANGLE,
        HOMING
    }

    private final TurretIO io;
    private State state = State.IDLE;
    // Local homed flag: some IO implementations may not provide a setter for
    // homing;
    // keep track locally once we home and/or zero the encoder.
    private boolean homed = false;

    // Soft limits in radians
    private final double minAngle;
    private final double maxAngle;

    public TurretSubsystem(TurretIO io, double minAngle, double maxAngle) {
        this.io = io;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
    }

    /**
     * Get the pivot simulation for testing.
     * 
     * @return The pivot simulation model
     */
    public SingleJointedArmSim getSimulation() {
        return io.getSimulation();
    }

    public void setManualVoltage(double volts) {
        state = State.MANUAL;
        io.setVoltage(volts);
    }

    public void moveToAngle(double radians) {
        state = State.MOVING_TO_ANGLE;
        double clamped = Math.max(minAngle, Math.min(maxAngle, radians));
        io.setPositionRadians(clamped);
    }

    public void stop() {
        state = State.IDLE;
        io.stop();
    }

    public double getAngle() {
        return io.getPositionRadians();
    }

    public boolean atAngle(double target, double tolerance) {
        return Math.abs(getAngle() - target) <= tolerance;
    }

    public void startHoming() {
        state = State.HOMING;
        // Implementation detail: drive slowly toward a limit switch
    }

    public State getState() {
        return state;
    }

    public void trackTarget(TurretTargetProvider targetProvider) {
        if (!targetProvider.hasTarget()) {
            // maybe switch to SEEKING state
            return;
        }
        state = State.TRACKING;
        double targetAngle = getAngle() + targetProvider.getTargetOffsetRadians();
        moveToAngle(targetAngle);
    }

    public boolean onTarget() {
        return true;
    }

    // --- Command factories ---

    /**
     * Manual control command. Reads a joystick axis (voltage or percent) from the
     * provided DoubleSupplier and calls {@link #setManualVoltage(double)} every
     * scheduler cycle. Useful for testing and fallback.
     */
    public Command manualControl(DoubleSupplier axisSupplier) {
        return run(() -> setManualVoltage(axisSupplier.getAsDouble()));
    }

    /**
     * Move to a fixed angle (radians). Completes when
     * {@link #atAngle(double,double)}
     * is true for the given tolerance.
     */
    public Command moveToAngleCommand(double targetRadians, double tolerance) {
        return Commands.sequence(
                runOnce(() -> moveToAngle(targetRadians)),
                // busy-wait until at angle
                run(() -> {
                }).until(() -> atAngle(targetRadians, tolerance)),
                runOnce(this::stop));
    }

    /**
     * Track a dynamic target provided by a {@link TurretTargetProvider}.
     * Continuously
     * updates the turret target. Ends when the command times out or when the
     * provider
     * reports no target for {@code noTargetMaxCycles} scheduler cycles.
     *
     * @param provider          the target provider
     * @param timeoutSeconds    overall timeout in seconds
     * @param noTargetMaxCycles number of consecutive cycles without a target before
     *                          ending
     */
    public Command trackTargetCommand(TurretTargetProvider provider, double timeoutSeconds, int noTargetMaxCycles) {
        final int[] missCount = new int[1];
        Command tracking = run(() -> {
            if (provider.hasTarget()) {
                trackTarget(provider);
                missCount[0] = 0;
            } else {
                missCount[0]++;
            }
        }).until(() -> missCount[0] >= noTargetMaxCycles).withTimeout(timeoutSeconds);

        return Commands.sequence(tracking, runOnce(this::stop));
    }

    /**
     * Homing command. Drives slowly toward a limit switch (forward or reverse).
     * When a limit switch is triggered the encoder is zeroed via
     * {@link TurretIO#setPositionRadians(double)}
     * (if implemented by the IO) and a local homed flag is set. Ends when homed or
     * on timeout.
     *
     * @param driveVoltage   small voltage to drive toward the limit switch (sign
     *                       used to pick direction)
     * @param timeoutSeconds overall timeout
     */
    public Command homeCommand(double driveVoltage, double timeoutSeconds) {
        final boolean[] driveTowardReverse = new boolean[1];
        // decide direction: if current angle nearer maxAngle, drive positive (forward),
        // else drive negative
        driveTowardReverse[0] = Math.abs(getAngle() - maxAngle) > Math.abs(getAngle() - minAngle);

        Command drive = run(() -> {
            double volts = driveTowardReverse[0] ? -Math.abs(driveVoltage) : Math.abs(driveVoltage);
            setManualVoltage(volts);
        }).until(() -> io.getForwardLimitSwitch() || io.getReverseLimitSwitch()).withTimeout(timeoutSeconds);

        Command finish = runOnce(() -> {
            // zero encoder (if the IO supports it via setPositionRadians)
            io.setPositionRadians(0.0);
            homed = true;
            stop();
        });

        return Commands.sequence(drive, finish);
    }

    /**
     * Seek command: sweep between soft limits (minAngle and maxAngle). Useful when
     * no target is visible. Continuously alternates between the two endpoints.
     */
    public Command seekCommand() {
        final boolean[] goingToMax = new boolean[] { true };
        final double tol = 0.02; // radians tolerance to flip direction

        return run(() -> {
            if (goingToMax[0]) {
                moveToAngle(maxAngle);
                if (atAngle(maxAngle, tol)) {
                    goingToMax[0] = false;
                }
            } else {
                moveToAngle(minAngle);
                if (atAngle(minAngle, tol)) {
                    goingToMax[0] = true;
                }
            }
        });
    }

    /**
     * Returns whether the turret has been homed. Checks local flag first, then IO.
     */
    public boolean isHomed() {
        return homed || io.isHomed();
    }
}
