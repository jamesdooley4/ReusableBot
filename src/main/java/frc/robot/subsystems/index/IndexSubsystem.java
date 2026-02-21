package frc.robot.subsystems.index;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeSubsystem;

// Subsystem for handling the ball/piece indexer mechanism
public class IndexSubsystem extends SubsystemBase {
    /**
     * Possible states for the indexer.
     */
    public enum State {
        IDLE,
        INTAKING,
        INDEXING,
        HOLDING,
        FEEDING,
        REVERSING,
        JAM_CLEARING
    }

    // IO abstraction for hardware
    private final IndexIO io;
    private State state = State.IDLE;

    /**
     * Create a new IndexSubsystem.
     * @param io IO implementation for hardware
     */
    public IndexSubsystem(IndexIO io) {
        this.io = io;
    }

    /**
     * Run the conveyor to intake a piece.
     */
    public void intake() {
        state = State.INTAKING;
        io.setConveyorVoltage(6.0);
    }

    /**
     * Move a piece from entry to mid or mid to exit.
     */
    public void index() {
        state = State.INDEXING;
        io.setConveyorVoltage(8.0);
    }

    /**
     * Feed a piece to the shooter.
     */
    public void feedToShooter() {
        state = State.FEEDING;
        io.setConveyorVoltage(10.0);
    }

    /**
     * Reverse the conveyor to clear jams.
     */
    public void reverse() {
        state = State.REVERSING;
        io.setConveyorVoltage(-6.0);
    }

    /**
     * Stop the conveyor and set state to idle.
     */
    public void stop() {
        state = State.IDLE;
        io.stopConveyor();
    }

    /**
     * @return true if a piece is detected at the entry sensor
     */
    public boolean hasPieceAtEntry() {
        return io.entrySensorTriggered();
    }

    /**
     * @return true if a piece is detected at the mid sensor
     */
    public boolean hasPieceAtMid() {
        return io.midSensorTriggered();
    }

    /**
     * @return true if a piece is detected at the exit sensor
     */
    public boolean hasPieceAtExit() {
        return io.exitSensorTriggered();
    }

    /**
     * @return the current state of the indexer
     */
    public State getState() {
        return state;
    }

    /**
     * Automatically index pieces using sensors.
     * @return a command that runs the indexer pipeline
     */
    public Command autoIndex() {
        return run(() -> {
            if (!hasPieceAtMid() && hasPieceAtEntry()) {
                intake();
            } else if (!hasPieceAtExit() && hasPieceAtMid()) {
                index();
            } else {
                stop();
            }
        }).until(() -> hasPieceAtEntry() && hasPieceAtMid() && hasPieceAtExit());
    }

    /**
     * Feed a piece to the launcher when shooter is ready.
     * @param shooterReady supplier indicating if shooter is ready
     * @return command to feed to launcher
     */
    public Command feedToLauncher(BooleanSupplier shooterReady) {
    return Commands.waitUntil(shooterReady).andThen(
        run(() -> feedToShooter())
            .until(() -> !hasPieceAtExit()),
        Commands.runOnce(this::stop, this));
    }

    /**
     * Clear a jam by reversing and then intaking.
     * @return command to clear jam
     */
    public Command clearJam() {
        return Commands.sequence(
                run(() -> reverse()).withTimeout(0.5),
                run(() -> intake()).withTimeout(0.5));
    }

    /**
     * Hold the indexer in position (stop motors).
     * @return command to hold position
     */
    public Command holdPosition() {
        return run(() -> stop());
    }

    /**
     * Run intake and indexer in parallel pipeline.
     * @param intakeSubsystem intake subsystem to coordinate
     * @return command to run pipeline
     */
    public Command intakeToIndexPipeline(IntakeSubsystem intakeSubsystem) {
        return Commands.parallel(
                intakeSubsystem.intake(),
                run(() -> {
                    if (!hasPieceAtMid() && hasPieceAtEntry()) {
                        intake();
                    } else if (!hasPieceAtExit() && hasPieceAtMid()) {
                        index();
                    } else {
                        stop();
                    }
                }).until(() -> hasPieceAtEntry() && hasPieceAtMid() && hasPieceAtExit()));
    }
}
