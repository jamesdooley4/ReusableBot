package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class ClimbSubsystem extends SubsystemBase {
    public enum State {
        IDLE,
        EXTENDING,
        HOOKING,
        LIFTING,
        BALANCING,
        LOCKING,
        STOWING,
        MANUAL
    }

    private final ClimbIO io;
    private State state = State.IDLE;

    private double targetWinchHeight = 0.0;
    private double targetArmAngle = 0.0;

    public ClimbSubsystem(ClimbIO io) {
        this.io = io;
    }

    // --- Command factories ---

    /**
     * Extend winch to a target height (meters). Ends when winch position reaches
     * the target (if position feedback exists) or an upper limit switch is hit.
     */
    public Command extendToBarCommand(double meters, double positionTolerance) {
        return Commands.sequence(
            runOnce(() -> extendToHeight(meters)),
            run(() -> {}).until(() -> io.atUpperLimit() || Math.abs(io.getWinchPositionMeters() - meters) <= positionTolerance),
            runOnce(() -> stop())
        );
    }

    /**
     * Rotate arm to hook angle and finish when hook is engaged or on timeout.
     */
    public Command hookBarCommand(double hookAngleRadians, double timeoutSeconds) {
        return Commands.sequence(
            runOnce(() -> rotateArmTo(hookAngleRadians)),
            run(() -> {}).until(() -> io.hookEngaged()).withTimeout(timeoutSeconds),
            runOnce(() -> stop())
        );
    }

    /**
     * Lift the robot by running the winch until reaching a target height or
     * a current spike is detected (indicating contact/strain). Ends when
     * the provided offGroundPredicate returns true.
     */
    public Command liftCommand(double targetHeightMeters, double currentSpikeThreshold, DoubleSupplier offGroundPredicate, double timeoutSeconds) {
        return Commands.sequence(
            run(() -> io.setWinchVoltage(10.0)).until(() -> io.getWinchPositionMeters() >= targetHeightMeters || io.getWinchCurrent() >= currentSpikeThreshold || offGroundPredicate.getAsDouble() > 0.5).withTimeout(timeoutSeconds),
            runOnce(() -> stop())
        );
    }

    /**
     * Balance command: periodically sample a tilt supplier and make small
     * winch adjustments to correct tilt. Runs until interrupted.
     *
     * @param tiltSupplier returns tilt in radians (positive one side down)
     * @param deadband radians inside which no action is taken
     * @param adjustVolts small voltage applied to correct tilt
     */
    public Command balanceCommand(DoubleSupplier tiltSupplier, double deadband, double adjustVolts) {
        return run(() -> {
            double tilt = tiltSupplier.getAsDouble();
            if (tilt > deadband) {
                // tilt positive -> lower one side: apply small positive winch
                io.setWinchVoltage(adjustVolts);
            } else if (tilt < -deadband) {
                io.setWinchVoltage(-adjustVolts);
            } else {
                io.setWinchVoltage(0.0);
            }
        });
    }

    /** Engage the ratchet and finish immediately. */
    public Command lockClimbCommand() {
        return runOnce(() -> lock());
    }

    /** Stow the climb mechanism and wait until winch is at lower limit or near zero. */
    public Command stowClimbCommand(double positionTolerance, double timeoutSeconds) {
        return Commands.sequence(
            runOnce(() -> stow()),
            run(() -> {}).until(() -> io.atLowerLimit() || Math.abs(io.getWinchPositionMeters()) <= positionTolerance).withTimeout(timeoutSeconds),
            runOnce(() -> stop())
        );
    }

    /** Manual climb command for testing/driver override. */
    public Command manualClimbCommand(DoubleSupplier winchAxisVolts, DoubleSupplier armAxisVolts) {
        return run(() -> manualControl(winchAxisVolts.getAsDouble(), armAxisVolts.getAsDouble()));
    }

    /**
     * Full auto-climb sequence: extend, hook, lift, balance, lock, stow.
     * Each stage uses reasonable defaults and should be tuned for your robot.
     */
    public Command autoClimbSequence( double extendMeters, double extendTol,
                                      double hookAngle, double hookTimeout,
                                      double liftTargetMeters, double liftCurrentSpike, DoubleSupplier offGroundPredicate, double liftTimeout,
                                      DoubleSupplier tiltSupplier, double balanceDeadband, double balanceAdjustVolts,
                                      double stowTol, double stowTimeout) {
        return Commands.sequence(
            extendToBarCommand(extendMeters, extendTol),
            hookBarCommand(hookAngle, hookTimeout),
            liftCommand(liftTargetMeters, liftCurrentSpike, offGroundPredicate, liftTimeout),
            balanceCommand(tiltSupplier, balanceDeadband, balanceAdjustVolts).withTimeout(5.0),
            lockClimbCommand(),
            stowClimbCommand(stowTol, stowTimeout)
        );
    }

    // ----- High-level intents -----

    public void extendToHeight(double meters) {
        state = State.EXTENDING;
        targetWinchHeight = meters;
        io.setWinchPositionMeters(meters);
    }

    public void rotateArmTo(double radians) {
        state = State.HOOKING;
        targetArmAngle = radians;
        io.setArmAngleRadians(radians);
    }

    public void lift() {
        state = State.LIFTING;
        io.setWinchVoltage(10.0);
    }

    public void balance() {
        state = State.BALANCING;
        // Implementation: small adjustments based on gyro or tilt sensor
    }

    public void lock() {
        state = State.LOCKING;
        io.setRatchetEngaged(true);
    }

    public void stow() {
        state = State.STOWING;
        io.setWinchPositionMeters(0.0);
        io.setArmAngleRadians(0.0);
    }

    public void manualControl(double winchVolts, double armVolts) {
        state = State.MANUAL;
        io.setWinchVoltage(winchVolts);
        io.setArmVoltage(armVolts);
    }

    public void stop() {
        state = State.IDLE;
        io.stopAll();
    }

    public State getState() {
        return state;
    }
}
