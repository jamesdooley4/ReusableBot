package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
    public enum State {
        IDLE,
        SPINNING_UP,
        AT_SPEED,
        SHOOTING,
        STOPPING
    }

    private final LauncherIO io;
    private State state = State.IDLE;

    private double targetVelocity = 0.0;
    private double targetHoodAngle = 0.0;

    public LauncherSubsystem(LauncherIO io) {
        this.io = io;
    }

    /**
     * Helper: is hood at target angle within tolerance
     */
    public boolean atHoodAngle(double target, double tolerance) {
        return Math.abs(io.getHoodPositionRadians() - target) <= tolerance;
    }

    public void setTargetVelocity(double radPerSec) {
        targetVelocity = radPerSec;
        state = State.SPINNING_UP;
        io.setFlywheelVelocity(radPerSec);
    }

    public void setHoodAngle(double radians) {
        targetHoodAngle = radians;
        io.setHoodPositionRadians(radians);
    }

    public boolean atSpeed(double tolerance) {
        return Math.abs(io.getFlywheelVelocityRadPerSec() - targetVelocity) <= tolerance;
    }

    public void shoot() {
        state = State.SHOOTING;
        // Indexer will feed; shooter just maintains velocity
    }

    public void stop() {
        state = State.STOPPING;
        io.stopFlywheel();
    }

    public State getState() {
        return state;
    }

    // --- Command factories ---

    /** Spin up flywheel to target velocity and finish when at speed. */
    public Command spinUpCommand(double velocityRadPerSec, double tolerance) {
        return Commands.sequence(
            runOnce(() -> setTargetVelocity(velocityRadPerSec)),
            // Wait until at speed
            run(() -> {}).until(() -> atSpeed(tolerance)),
            runOnce(() -> state = State.AT_SPEED)
        );
    }

    /** Move hood to angle and finish when within tolerance. */
    public Command setHoodAngleCommand(double targetRadians, double tolerance) {
        return Commands.sequence(
            runOnce(() -> setHoodAngle(targetRadians)),
            run(() -> {}).until(() -> atHoodAngle(targetRadians, tolerance)),
            runOnce(() -> {})
        );
    }

    /**
     * Waits for the shooter to reach speed (tolerance) then tells the provided
     * IndexSubsystem to feed. Uses the IndexSubsystem.feedToShooter API which
     * accepts a shooter-ready command.
     */
    public Command shootWhenReadyCommand(IndexSubsystem indexSubsystem, double speedTolerance) {
        Command shooterReady = run(() -> {}).until(() -> atSpeed(speedTolerance));
        return indexSubsystem.feedToShooter(shooterReady);
    }

    /**
     * Auto shot: given functions (vision->hood angle, vision->velocity) and a
     * distance value, configure hood and flywheel, spin up, then shoot when ready.
     */
    public Command autoShotCommand(double visionDistanceMeters, java.util.function.DoubleUnaryOperator distanceToHoodAngle,
            java.util.function.DoubleUnaryOperator distanceToFlywheelVelocity, IndexSubsystem indexSubsystem, double speedTolerance) {
        double hoodAngle = distanceToHoodAngle.applyAsDouble(visionDistanceMeters);
        double flywheelVel = distanceToFlywheelVelocity.applyAsDouble(visionDistanceMeters);

        return Commands.sequence(
            setHoodAngleCommand(hoodAngle, 0.02),
            spinUpCommand(flywheelVel, speedTolerance),
            shootWhenReadyCommand(indexSubsystem, speedTolerance)
        );
    }

    /** Stop shooter (flywheel) and attempt to hold hood position. */
    public Command stopShooterCommand() {
        return runOnce(() -> {
            io.stopFlywheel();
            // keep hood where it is by commanding current position (best-effort)
            io.setHoodPositionRadians(io.getHoodPositionRadians());
            state = State.STOPPING;
        });
    }

    /**
     * Characterize the shooter by running a set of voltage steps for a fixed
     * duration each (useful for SysId data collection). The command will run
     * each step in sequence and stop the flywheel at the end.
     *
     * @param voltages array of voltages to apply in sequence
     * @param stepDurationSeconds how long to hold each voltage
     */
    public Command characterizeShooterCommand(double[] voltages, double stepDurationSeconds) {
        java.util.List<Command> steps = new java.util.ArrayList<>();
        for (double v : voltages) {
            steps.add(run(() -> io.setFlywheelVoltage(v)).withTimeout(stepDurationSeconds));
        }
        steps.add(runOnce(() -> io.stopFlywheel()));
        return Commands.sequence(steps.toArray(new Command[0]));
    }
}
