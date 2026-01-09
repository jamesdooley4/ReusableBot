package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexSubsystem extends SubsystemBase {
    public enum State {
        IDLE,
        INTAKING,
        INDEXING,
        HOLDING,
        FEEDING,
        REVERSING,
        JAM_CLEARING
    }

    private final IndexIO io;
    private State state = State.IDLE;

    public IndexSubsystem(IndexIO io) {
        this.io = io;
    }

    public void intake() {
        state = State.INTAKING;
        io.setConveyorVoltage(6.0);
    }

    public void index() {
        state = State.INDEXING;
        io.setConveyorVoltage(8.0);
    }

    public void feedToShooter() {
        state = State.FEEDING;
        io.setConveyorVoltage(10.0);
    }

    public void reverse() {
        state = State.REVERSING;
        io.setConveyorVoltage(-6.0);
    }

    public void stop() {
        state = State.IDLE;
        io.stopConveyor();
    }

    public boolean hasPieceAtEntry() { return io.entrySensorTriggered(); }
    public boolean hasPieceAtMid() { return io.midSensorTriggered(); }
    public boolean hasPieceAtExit() { return io.exitSensorTriggered(); }

    public State getState() { return state; }

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

    public Command feedToShooter(Command shooterReadyCommand) {
        return Commands.sequence(
            shooterReadyCommand,
            run(() -> feedToShooter())
                .until(() -> !hasPieceAtExit()),
            Commands.runOnce(this::stop, this)
        );
    }

    public Command clearJam() {
        return Commands.sequence(
            run(() -> reverse()).withTimeout(0.5),
            run(() -> intake()).withTimeout(0.5)
        );
    }

    public Command holdPosition() {
        return run(() -> stop());
    }

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
            }).until(() -> hasPieceAtEntry() && hasPieceAtMid() && hasPieceAtExit())
        );
    }
}